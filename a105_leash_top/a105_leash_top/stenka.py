#!/usr/bin/env python3
# ROS 2 Humble: RGBD → cmd_vel. Следование по левому краю белой дороги с отступом и фоллбэками.
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

class LeftEdgeFollowerNode(Node):
    def __init__(self):
        super().__init__("left_edge_follower")
        self.bridge = CvBridge()

        self.declare_parameters("", [
            # Топики
            ("rgb_topic", "/camera/color/image_raw"),
            ("depth_topic", "/camera/depth/image_raw"),
            ("camera_info_topic", "/camera/color/camera_info"),
            ("enable_debug", True),

            # Сегментация белого
            ("hsv_S_max", 60),
            ("hsv_V_min", 180),
            ("morph_kernel", 3),
            ("roi_ymin_frac", 0.10),
            ("roi_ymax_frac", 0.75),
            ("near_cut_m", 0.6),

            # Левый край
            ("step_px", 6),
            ("edge_offset_px", 45),         # отступ внутрь полотна от левого края
            ("edge_smooth_alpha", 0.4),

            # Цель и контроль
            ("lookahead_m", 0.45),
            ("tgt_depth_tol_m", 1.0),
            ("min_v", 0.05), ("max_v", 0.18),
            ("w_gain", 3.0), ("w_max", 2.2),
            ("Ld_min", 0.25), ("Ld_alpha_k", 1.6), ("v_alpha_k", 2.2),

            # Защита вперёд
            ("forward_valid_ratio", 0.20),

            # Поиск края, если маски мало
            ("search_v", 0.04),
            ("search_w", 0.5),

            # Ползок при сомнительной видимости
            ("creep_v", 0.03),
        ])

        rgb_sub   = message_filters.Subscriber(self, Image, self.get_parameter("rgb_topic").value,
                                               qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, self.get_parameter("depth_topic").value,
                                               qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 30, 0.12)
        self.sync.registerCallback(self.synced_cb)

        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                 self.info_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.enable_debug = bool(self.get_parameter("enable_debug").value)
        if self.enable_debug:
            self.mask_pub = self.create_publisher(Image, "white_mask", qos_profile_sensor_data)
            self.viz_pub  = self.create_publisher(Image, "edge_viz", qos_profile_sensor_data)

        self.fx = None; self.cx = None
        self._x_prev = None

    # --- Intrinsics ---
    def info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0]); self.cx = float(msg.k[2])

    # --- Segmentation ---
    def segment_white(self, bgr, depth_m):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            (0, 0, int(self.get_parameter("hsv_V_min").value)),
            (179, int(self.get_parameter("hsv_S_max").value), 255)
        )
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

    # --- Left edge with smoothing and offset ---
    def left_edge_points(self, mask, ymin, ymax):
        h, w = mask.shape
        step = int(self.get_parameter("step_px").value)
        offset = int(self.get_parameter("edge_offset_px").value)
        alpha = float(self.get_parameter("edge_smooth_alpha").value)

        xs_idx = np.arange(w, dtype=np.int32)
        pts = []
        for y in range(min(h-1, ymax-1), ymin-1, -step):
            line = mask[y]
            xs = xs_idx[line > 0]
            if xs.size == 0:
                continue
            x_left = int(xs[0])
            if self._x_prev is None:
                x_s = float(x_left)
            else:
                x_s = alpha * float(self._x_prev) + (1.0 - alpha) * float(x_left)
            self._x_prev = x_s
            x_in = int(clamp(round(x_s) + offset, 0, w-1))
            pts.append((x_in, y))
        return pts

    # --- Depth helpers ---
    def depth_at(self, depth, y, x):
        h, w = depth.shape
        x = int(clamp(x, 0, w-1)); y = int(clamp(y, 0, h-1))
        z = depth[y, x]
        return float(z) * 0.001 if depth.dtype == np.uint16 else float(z)

    # --- Target selection ---
    def pick_target(self, pts, depth_img, z_des, tol):
        if not pts:
            return None
        best = None; best_err = 1e9
        for (x, y) in pts:
            z = self.depth_at(depth_img, y, x)
            if not np.isfinite(z) or z <= 0.05:
                continue
            err = abs(z - z_des)
            if err < best_err and err <= tol:
                best = (x, y); best_err = err
        if best is None:
            best = max(pts, key=lambda p: p[1])  # ближняя по y
        return best

    # --- Pure Pursuit approx in camera frame ---
    def control_pp(self, u, v, depth_img, Ld0, width_px):
        z = self.depth_at(depth_img, v, u)
        if not np.isfinite(z) or z <= 0.05:
            z = Ld0
        # Подстраховка по intrinsics, если CameraInfo не пришёл
        fx = self.fx if self.fx is not None else 525.0
        cx = self.cx if self.cx is not None else (width_px * 0.5)

        x = (u - cx) * z / fx
        alpha = math.atan2(x, z)

        Ld = max(float(self.get_parameter("Ld_min").value),
                 Ld0 * (1.0 - float(self.get_parameter("Ld_alpha_k").value) * min(1.0, abs(alpha))))
        v = clamp(
            float(self.get_parameter("max_v").value) * (1.0 - float(self.get_parameter("v_alpha_k").value) * abs(alpha)),
            float(self.get_parameter("min_v").value),
            float(self.get_parameter("max_v").value)
        )
        w = - float(self.get_parameter("w_gain").value) * 2.0 * v * math.sin(alpha) / max(Ld, 1e-3)
        w = clamp(w, -float(self.get_parameter("w_max").value), float(self.get_parameter("w_max").value))
        return v, w, alpha

    # --- Forward validity check ---
    def forward_valid(self, mask, depth_m, tgt, ymin):
        if tgt is None or depth_m is None:
            return True
        x, y = tgt
        x0 = clamp(x - 20, 0, mask.shape[1]-1)
        x1 = clamp(x + 20, 0, mask.shape[1]-1)
        band = slice(max(ymin, y - 50), y)
        droi = depth_m[band, int(x0):int(x1)]
        if droi.size == 0:
            return False
        valid = np.isfinite(droi) & (droi > 0.05)
        return valid.mean() >= float(self.get_parameter("forward_valid_ratio").value)

    # --- Main ---
    def synced_cb(self, rgb_msg: Image, depth_msg: Image):
        bgr   = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        depth_m = depth.astype(np.float32) * 0.001 if depth.dtype == np.uint16 else depth.astype(np.float32)

        mask, ymin, ymax = self.segment_white(bgr, depth_m)
        pts = self.left_edge_points(mask, ymin, ymax)

        twist = Twist()
        tgt = None

        if pts:
            z_des = float(self.get_parameter("lookahead_m").value)
            tol   = float(self.get_parameter("tgt_depth_tol_m").value)

            tgt = self.pick_target(pts, depth, z_des, tol)
            ok_forward = self.forward_valid(mask, depth_m, tgt, ymin)

            v_cmd, w_cmd, _ = self.control_pp(tgt[0], tgt[1], depth, z_des, bgr.shape[1])

            if ok_forward:
                twist.linear.x = v_cmd
                twist.angular.z = w_cmd
            else:
                twist.linear.x = max(float(self.get_parameter("creep_v").value),
                                     float(self.get_parameter("min_v").value))
                twist.angular.z = w_cmd
        else:
            twist.linear.x = float(self.get_parameter("search_v").value)
            twist.angular.z = float(self.get_parameter("search_w").value)

        self.cmd_pub.publish(twist)

        # Диагностика
        try:
            self.get_logger().info(
                f"pts={len(pts)} tgt={tgt} v={twist.linear.x:.3f} w={twist.angular.z:.3f}"
            )
        except:
            pass

        # Отладочные изображения
        if self.enable_debug:
            try:
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
                viz = bgr.copy()
                for (x, y) in pts:
                    cv2.circle(viz, (x, y), 2, (0, 0, 255), -1)
                if tgt:
                    cv2.circle(viz, (tgt[0], tgt[1]), 5, (255, 0, 0), -1)
                self.viz_pub.publish(self.bridge.cv2_to_imgmsg(viz, "bgr8"))
            except:
                pass

def main():
    rclpy.init()
    n = LeftEdgeFollowerNode()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
