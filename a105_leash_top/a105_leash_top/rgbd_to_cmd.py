#!/usr/bin/env python3
# ROS 2 Humble: RGBD → cmd_vel. DT-гребень + коридор/ползок + автоповорот влево при тупике.
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import message_filters

def clamp(x, a, b): return a if x < a else b if x > b else x

class WhiteRoadNode(Node):
    def __init__(self):
        super().__init__("white_road_node")
        self.bridge = CvBridge()

        self.declare_parameters("", [
            ("rgb_topic", "/camera/color/image_raw"),
            ("depth_topic", "/camera/depth/image_raw"),
            ("camera_info_topic", "/camera/color/camera_info"),
            ("enable_debug", True),

            # сегментация
            ("hsv_S_max", 60), ("hsv_V_min", 180),
            ("morph_kernel", 5),
            ("roi_ymin_frac", 0.01),
            ("roi_ymax_frac", 0.77),
            ("near_cut_m", 0.020),

            # резерв по строкам
            ("step_px", 8), ("min_width_px", 12),

            # гребень distance transform
            ("use_ridge_centerline", True),
            ("ridge_step_px", 4),
            ("ridge_search_margin_px", 100),
            ("ridge_min_dist_px", 2.0),
            ("ridge_init_band_px", 120),
            ("ridge_smooth_alpha", 0.25),

            # цель и контроль
            ("lookahead_m", 0.37),
            ("min_v", 0.05), ("max_v", 0.13),
            ("w_gain", 2.5), ("w_max", 2.3),
            ("Ld_min", 0.25), ("Ld_alpha_k", 2.3), ("v_alpha_k", 2.0),
            ("tgt_depth_gain", 1.25),

            # защита «в пустоту» + ползок
            ("forward_band_px", 60),
            ("forward_mask_min_coverage", 0.20),
            ("forward_depth_valid_ratio", 0.60),
            ("void_depth_jump", 0.90),
            ("alpha_relax_k", 1.7),
            ("corridor_px", 40),
            ("creep_on_turn", True),
            ("creep_alpha_min", 0.4),
            ("creep_v", 0.04),

            # тупик → крутиться влево
            ("dead_reach_px", 60),          # минимальная достигнутая «глубина» центролинии в пикселях
            ("dead_use_guard_void", True),  # учитывать guard_void в критерии тупика
            ("dead_turn_w", 0.3),           # рад/с, положительный = влево
        ])

        rgb_sub = message_filters.Subscriber(self, Image, self.get_parameter("rgb_topic").value,
                                             qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, self.get_parameter("depth_topic").value,
                                               qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 30, 0.15)
        self.sync.registerCallback(self.synced_cb)

        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                 self.info_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.enable_debug = bool(self.get_parameter("enable_debug").value)
        if self.enable_debug:
            self.mask_pub    = self.create_publisher(Image, "white_mask", qos_profile_sensor_data)
            self.viz_pub     = self.create_publisher(Image, "centerline_viz", qos_profile_sensor_data)
            self.overlay_pub = self.create_publisher(Image, "road_overlay", qos_profile_sensor_data)

        self.fx = None; self.cx = None

        # состояние «крутимся влево»
        self.spin_left_mode = False

    def info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0]); self.cx = float(msg.k[2])

    def segment_white(self, bgr, depth_m):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           (0, 0, int(self.get_parameter("hsv_V_min").value)),
                           (179, int(self.get_parameter("hsv_S_max").value), 255))
        h, w = mask.shape
        ymin = int(h * float(self.get_parameter("roi_ymin_frac").value))
        ymax = int(h * float(self.get_parameter("roi_ymax_frac").value))
        ymin = clamp(ymin, 0, h-1); ymax = clamp(ymax, ymin+1, h)
        mask[:ymin, :] = 0; mask[ymax:, :] = 0

        near = float(self.get_parameter("near_cut_m").value)
        if depth_m is not None:
            mask[(depth_m > 0.0) & (depth_m < near)] = 0

        k = max(3, int(self.get_parameter("morph_kernel").value) | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        if num > 1:
            biggest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            mask = np.where(labels == biggest, 255, 0).astype(np.uint8)
        return mask, ymin, ymax

    def centerline_points(self, mask, ymin, ymax):
        h, w = mask.shape
        step_px = int(self.get_parameter("step_px").value)
        min_width = int(self.get_parameter("min_width_px").value)
        ys = range(min(h-1, ymax-1), ymin-1, -step_px)
        centers = []
        xs_idx = np.arange(w, dtype=np.int32)
        for y in ys:
            line = mask[y]; xs = xs_idx[line > 0]
            if xs.size < min_width: continue
            xL = int(xs[0]); xR = int(xs[-1])
            if (xR - xL) < min_width: continue
            centers.append(((xL + xR)//2, y))
        return centers

    def ridge_centerline(self, mask, ymin, ymax):
        road = (mask > 0).astype(np.uint8)
        if road.sum() == 0:
            return [], None
        dist = cv2.distanceTransform(road, cv2.DIST_L2, 3)

        h, w = mask.shape
        step    = int(self.get_parameter("ridge_step_px").value)
        margin  = int(self.get_parameter("ridge_search_margin_px").value)
        min_d   = float(self.get_parameter("ridge_min_dist_px").value)
        init_bd = int(self.get_parameter("ridge_init_band_px").value)
        smooth  = float(self.get_parameter("ridge_smooth_alpha").value)

        y_start = int(min(ymax - 1, h - 1))
        y0 = int(max(ymin, y_start - init_bd))
        band = dist[y0:y_start, :]
        if band.size == 0 or band.max() <= 0:
            return [], dist

        x_prev = float(int(np.argmax(band.max(axis=0))))
        centers = []
        for y in range(y_start, ymin - 1, -step):
            xl = int(max(0, round(x_prev) - margin))
            xr = int(min(w, round(x_prev) + margin + 1))
            if xr - xl < 3: continue
            row = dist[int(y), xl:xr]
            m = float(row.max()) if row.size else 0.0
            if m < min_d: continue
            thr = 0.7 * m
            idx = np.arange(xl, xr, dtype=np.float32)
            wts = np.maximum(row - thr, 0.0) ** 2
            if wts.sum() > 1e-6:
                x_new = float((idx * wts).sum() / wts.sum())
            else:
                k = int(np.argmax(row)); x_new = float(xl + k)
                if 0 < k < row.size - 1:
                    y0_, y1_, y2_ = float(row[k - 1]), float(row[k]), float(row[k + 1])
                    denom = 2.0 * (y0_ - 2.0 * y1_ + y2_)
                    if abs(denom) > 1e-6:
                        x_new = float(xl + (k + 0.5 * (y0_ - y2_) / denom))
            x_prev = float(smooth * x_prev + (1.0 - smooth) * x_new)
            centers.append((int(round(x_prev)), int(y)))
        return centers, dist

    def depth_at(self, depth, y, x):
        h, w = depth.shape
        x = int(clamp(x, 0, w-1)); y = int(clamp(y, 0, h-1))
        z = depth[y, x]
        return float(z) * 0.001 if depth.dtype == np.uint16 else float(z)

    def guard_void(self, mask, depth_m, tgt, ymin, ymax, alpha):
        if tgt is None:
            return False

        x, y = int(tgt[0]), int(tgt[1])

        # релаксация
        a = min(abs(alpha), 1.2)
        relax = 1.0 - float(self.get_parameter("alpha_relax_k").value) * (a / 1.2)

        band_h = int(max(20, round(float(self.get_parameter("forward_band_px").value) * relax)))
        half_w = int(max(15, round(float(self.get_parameter("corridor_px").value) * relax)))
        cov_min = max(0.05, float(self.get_parameter("forward_mask_min_coverage").value) * relax)
        val_min = max(0.3,  float(self.get_parameter("forward_depth_valid_ratio").value) * relax)

        H, W = mask.shape
        x0 = int(max(0, x - half_w))
        x1 = int(min(W, x + half_w))
        y0 = int(max(ymin, y - band_h))
        y1 = int(min(y,   ymax))

        if x1 <= x0 or y1 <= y0:
            return False

        # покрытие маской
        mroi = mask[y0:y1, x0:x1]
        cover = (mroi > 0).mean() if mroi.size else 0.0
        if cover < cov_min:
            return False

        # глубина
        if depth_m is None:
            return True
        droi = depth_m[y0:y1, x0:x1]
        if droi.size == 0:
            return False

        valid = np.isfinite(droi) & (droi > 0.05)
        if valid.mean() < val_min:
            return False

        h = droi.shape[0]
        h2 = h // 2
        near = droi[h2:h][valid[h2:h]] if h2 < h else droi[h-1:h][valid[h-1:h]]
        far  = droi[0:h2][valid[0:h2]] if h2 > 0 else droi[0:1][valid[0:1]]
        if near.size and far.size:
            near_med = float(np.median(near))
            far_med  = float(np.median(far))
            if np.isfinite(near_med) and np.isfinite(far_med) and \
            (far_med - near_med) > float(self.get_parameter("void_depth_jump").value):
                return False

        return True


    # критерий «путь валиден» + возврат выбранной цели
    def path_viable_and_target(self, centers, depth, mask, ymin, ymax, depth_m):
        if not centers: return False, None, 0.0
        # глубина «достигнутой» центролинии
        ys = [p[1] for p in centers]
        reach_px = int(max(ys) - min(ys))
        if reach_px < int(self.get_parameter("dead_reach_px").value):
            return False, None, 0.0

        Ld = float(self.get_parameter("lookahead_m").value)
        wantZ = Ld * float(self.get_parameter("tgt_depth_gain").value)
        tgt = min(centers, key=lambda p: abs(self.depth_at(depth, p[1], p[0]) - wantZ))

        z_est = self.depth_at(depth, tgt[1], tgt[0]); z_est = z_est if np.isfinite(z_est) and z_est > 0.05 else Ld
        x_est = (tgt[0] - self.cx) * z_est / self.fx
        alpha = math.atan2(x_est, z_est)

        if bool(self.get_parameter("dead_use_guard_void").value):
            if not self.guard_void(mask, depth_m, tgt, ymin, ymax, alpha):
                return False, tgt, alpha
        return True, tgt, alpha

    def control_pp(self, u, v, depth_img, Ld0, v_min, v_max):
        z = self.depth_at(depth_img, v, u);  z = z if np.isfinite(z) and z > 0.05 else Ld0
        x = (u - self.cx) * z / self.fx
        alpha = math.atan2(x, z)
        Ld = max(float(self.get_parameter("Ld_min").value),
                 Ld0 * (1.0 - float(self.get_parameter("Ld_alpha_k").value) * min(1.0, abs(alpha))))
        v = clamp(v_max * (1.0 - float(self.get_parameter("v_alpha_k").value) * abs(alpha)),
                  v_min, v_max)
        w = - float(self.get_parameter("w_gain").value) * 2.0 * v * math.sin(alpha) / max(Ld, 1e-3)
        w = clamp(w, -float(self.get_parameter("w_max").value), float(self.get_parameter("w_max").value))
        return v, w, alpha

    def build_overlay(self, bgr, mask, centers, tgt, ymin, ymax, dist=None):
        ov = bgr.copy()
        cm = np.zeros_like(bgr); cm[..., 1] = mask
        ov = cv2.addWeighted(ov, 1.0, cm, 0.35, 0.0)
        if dist is not None:
            dvis = cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            dvis = cv2.applyColorMap(dvis, cv2.COLORMAP_TURBO)
            ov = cv2.addWeighted(ov, 0.8, dvis, 0.2, 0.0)
        cv2.line(ov, (0, ymin), (ov.shape[1]-1, ymin), (255, 255, 0), 1)
        cv2.line(ov, (0, ymax), (ov.shape[1]-1, ymax), (255, 255, 0), 1)
        if len(centers) > 1:
            pts = np.array(centers, dtype=np.int32)
            cv2.polylines(ov, [pts], False, (0, 0, 255), 2)
        for (x, y) in centers:
            cv2.circle(ov, (x, y), 2, (0, 0, 255), -1)
        if tgt is not None:
            cv2.circle(ov, (tgt[0], tgt[1]), 6, (255, 0, 0), -1)
        return ov

    def synced_cb(self, rgb_msg: Image, depth_msg: Image):
        if self.fx is None: 
            return

        bgr   = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        depth_m = depth.astype(np.float32) * 0.001 if depth.dtype == np.uint16 else depth.astype(np.float32)

        mask, ymin, ymax = self.segment_white(bgr, depth_m)
        dist = None
        if bool(self.get_parameter("use_ridge_centerline").value):
            centers, dist = self.ridge_centerline(mask, ymin, ymax)
        else:
            centers = self.centerline_points(mask, ymin, ymax)

        viable, tgt, alpha = self.path_viable_and_target(centers, depth, mask, ymin, ymax, depth_m)

        tw = Twist()

        # режим «крутимся влево, пока не станет валидно»
        if self.spin_left_mode:
            if viable and tgt is not None:
                self.spin_left_mode = False
            else:
                tw.angular.z = float(self.get_parameter("dead_turn_w").value)  # влево
                self.cmd_pub.publish(tw)
                if self.enable_debug:
                    try:
                        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
                        viz = np.zeros_like(bgr)
                        for (x, y) in centers: cv2.circle(viz, (x, y), 3, (0, 0, 255), -1)
                        self.viz_pub.publish(self.bridge.cv2_to_imgmsg(viz, "bgr8"))
                        overlay = self.build_overlay(bgr, mask, centers, None, ymin, ymax, dist)
                        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))
                    except: pass
                return

        if viable and tgt is not None:
            Ld = float(self.get_parameter("lookahead_m").value)
            v_cmd, w_cmd, _ = self.control_pp(tgt[0], tgt[1], depth,
                                              Ld, float(self.get_parameter("min_v").value),
                                              float(self.get_parameter("max_v").value))
            tw.linear.x, tw.angular.z = v_cmd, w_cmd
        else:
            # вход в режим вращения влево
            self.spin_left_mode = True
            tw.angular.z = float(self.get_parameter("dead_turn_w").value)

        self.cmd_pub.publish(tw)

        if self.enable_debug:
            try: self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            except: pass
            viz = np.zeros_like(bgr)
            for (x, y) in centers: cv2.circle(viz, (x, y), 3, (0, 0, 255), -1)
            if tgt is not None: cv2.circle(viz, (tgt[0], tgt[1]), 6, (255, 0, 0), -1)
            try:
                self.viz_pub.publish(self.bridge.cv2_to_imgmsg(viz, "bgr8"))
                overlay = self.build_overlay(bgr, mask, centers, tgt, ymin, ymax, dist)
                self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))
            except: pass

def main():
    rclpy.init()
    n = WhiteRoadNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
