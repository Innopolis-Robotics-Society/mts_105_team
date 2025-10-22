import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthCanny(Node):
    def __init__(self):
        super().__init__("depth_canny")
        self.declare_parameters("", [
            ("depth_topic", "/camera/depth/image_raw"),
            ("edges_topic", "/camera/depth/depth_edges"),
            ("range_max", 8.0),
            ("bilateral_d", 3),
            ("bilateral_sigma", 20.0),
            ("canny_lo", 10),
            ("canny_hi", 40),
            ("exclude_width_px",  200),   # ширина прямоугольника (пиксели)
            ("exclude_height_px", 150),   # высота от нижнего края (пиксели)
        ])
        gp = self.get_parameter
        self.depth_topic   = gp("depth_topic").value
        self.edges_topic   = gp("edges_topic").value
        self.range_max   = float(gp("range_max").value)
        self.b_d           = int(gp("bilateral_d").value)
        self.b_sigma       = float(gp("bilateral_sigma").value)
        self.canny_lo      = int(gp("canny_lo").value)
        self.canny_hi      = int(gp("canny_hi").value)
        self.exc_w = int(self.get_parameter("exclude_width_px").value)
        self.exc_h = int(self.get_parameter("exclude_height_px").value)


        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, self.depth_topic, self.on_depth, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(
            Image, self.edges_topic, qos_profile_sensor_data
        )

    def on_depth(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')  # 16UC1 мм или 32FC1 м
        
        # H, W = img.shape
        # w = max(0, min(W, self.exc_w))
        # h = max(0, min(H, self.exc_h))

        # x0 = max(0, (W - w) // 2)
        # x1 = min(W, x0 + w)
        # y0 = max(0, H - h)   # от нижнего края вверх
        # y1 = H

        # if x1 > x0 and y1 > y0:
        #     img[y0:y1, x0:x1] = 0.0
        
        #if img.dtype == np.uint16:
        depth_m = img.astype(np.float32) * 1e-3
        depth_m = np.clip(depth_m, 0.0, self.range_max)
        depth_m = cv2.bilateralFilter(depth_m, self.b_d, self.b_sigma, self.b_sigma)
        d8 = cv2.convertScaleAbs(depth_m, alpha=255.0/max(self.range_max,1e-6))
        d8 = cv2.GaussianBlur(d8, (3,3), 0)
        edges = cv2.Canny(d8, self.canny_lo, self.canny_hi)
        # маска краёв → оставить исходные миллиметры, прочее = 0
        masked = np.where(edges>0, img, 0).astype(np.uint16)
        # публикуем
        m_msg = self.bridge.cv2_to_imgmsg(masked, encoding='16UC1'); m_msg.header = msg.header
        self.pub.publish(m_msg)

        # else:  # 32FC1 метры
        #     depth_m = img.astype(np.float32)
        #     depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
        #     depth_m = np.clip(depth_m, 0.0, self.range_max)
        #     depth_m = cv2.bilateralFilter(depth_m, self.b_d, self.b_sigma, self.b_sigma)
        #     d8 = cv2.convertScaleAbs(depth_m, alpha=255.0/max(self.range_max,1e-6))
        #     d8 = cv2.GaussianBlur(d8, (3,3), 0)
        #     edges = cv2.Canny(d8, self.canny_lo, self.canny_hi)
        #     # где нет краёв → NaN, где есть → исходная глубина
        #     masked = depth_m.copy()
        #     masked[edges==0] = np.nan
        #     # публикуем
        #     m_msg = self.bridge.cv2_to_imgmsg(masked, encoding='32FC1');  m_msg.header = msg.header
        #     self.pub.publish(m_msg)

def main():
    rclpy.init()
    rclpy.spin(DepthCanny())
    rclpy.shutdown()
