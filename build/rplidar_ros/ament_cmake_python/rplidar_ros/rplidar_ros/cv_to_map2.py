#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt


class LaneMapPublisher(Node):
    def __init__(self):
        super().__init__('lane_map_publisher')
        self.occupancy_grid_size = 200
        self.start_time = None
        self.vertical_offset = 5
        self.height_offset = 58
        self.captured_width = 70
        self.captured_height = 50
        self.frame_rate = 8.0

        self.preview_pub = self.create_publisher(Image, "/preview", 10)
        self.image_pub = self.create_publisher(OccupancyGrid, "/lane_map", 10)

        self.header = Header()
        self.header.frame_id = "base_link"

        self.map_info = MapMetaData()
        self.map_info.width = self.occupancy_grid_size
        self.map_info.height = self.occupancy_grid_size  # Set to occupancy_grid_size
        self.map_info.resolution = 0.1
        self.map_info.origin = Pose()
        self.map_info.origin.position.x = -10.0
        self.map_info.origin.position.y = -10.0

        # self.cam = cv2.VideoCapture('gst-launch-1.0 souphttpsrc location=http://127.0.0.1:8080 ! decodebin ! videoconvert ! appsink')  # Set the camera index as needed
        self.cam = cv2.VideoCapture('ttp://127.0.0.1')
        self.imshowout = None

        self.create_timer(1.0 / self.frame_rate, self.camera_callback)

        self.transform = PerspectiveTransform(5)  # Initialize with a camera angle

    def camera_callback(self):
        read_success, image = self.cam.read()

        if not read_success:
            self.get_logger().warning("Camera read problem")
            return

        image = cv2.GaussianBlur(image, (7, 7), 0)
        pre_or_post_filtered_image = grass_filter(image)

        height, width = image.shape[:2]
        region_of_interest_vertices = [
            (0, height),
            (width / 2, height / 2 + 70),
            (width, height),
        ]

        gray_image = pre_or_post_filtered_image
        cropped_image = region_of_interest(gray_image, np.array([region_of_interest_vertices], np.int32))

        blurred = cv2.GaussianBlur(cropped_image, (7, 7), 0)
        blurred[blurred < 245] = 0

        perspective_crop = self.transform.trim_top_border(blurred)
        perspective_warp = self.transform.convert_to_flat(perspective_crop)

        if self.imshowout is None:
            self.imshowout = plt.imshow(perspective_warp)
            plt.show(block=False)
            plt.pause(0.001)
        else:
            self.imshowout.set_data(perspective_warp)
            plt.show(block=False)
            plt.pause(0.001)

        self.numpy_to_occupancy_grid(perspective_warp)

    def numpy_to_occupancy_grid(self, data_map):
        data_map = cv2.dilate(data_map, (5, 5), iterations=3)
        data_map = cv2.resize(data_map, dsize=(self.captured_width, self.captured_height), interpolation=cv2.INTER_LINEAR) / 2
        data_map = data_map[self.vertical_offset:, :]
        data_map = cv2.copyMakeBorder(data_map, self.vertical_offset + self.height_offset, 200 - self.captured_height - self.height_offset, (200 - self.captured_width) // 2, (200 - self.captured_width) // 2, cv2.BORDER_CONSTANT, value=0)
        data_map = cv2.flip(data_map, 0)
        data_map = cv2.rotate(data_map, cv2.ROTATE_90_COUNTERCLOCKWISE)
        flattened = list(data_map.flatten().astype(int))

        self.header.stamp = self.get_clock().now().to_msg()
        msg = OccupancyGrid(header=self.header, info=self.map_info, data=flattened)
        self.image_pub.publish(msg)


class PerspectiveTransform:
    def __init__(self, camera_angle):
        self.top_trim_ratio = 0.0
        self.camera_angle = camera_angle
        self.horizontal_corner_cut_ratio = 0.26
        self.output_img_shape_x = 640
        self.output_img_shape_y = 480

    def calc_trim_ratio(self):
        return 0.25 * (self.camera_angle) / 100

    def trim_top_border(self, img):
        y_min = int(img.shape[0] * self.calc_trim_ratio())
        y_max = img.shape[0]
        x_min = 0
        x_max = img.shape[1]
        return img[y_min:y_max, x_min:x_max]

    def convert_to_flat(self, img):
        top_left = (int(img.shape[1] * self.horizontal_corner_cut_ratio), img.shape[0])
        top_right = (int(img.shape[1] - img.shape[1] * self.horizontal_corner_cut_ratio), img.shape[0])
        bottom_left = (0, 0)
        bottom_right = (img.shape[1], 0)

        src_pts = np.float32([top_left, top_right, bottom_left, bottom_right])
        dest_pts = np.float32([[0, self.output_img_shape_y], [self.output_img_shape_x, self.output_img_shape_y], [0, 0], [self.output_img_shape_x, 0]])

        matrix = cv2.getPerspectiveTransform(src_pts, dest_pts)
        output = cv2.warpPerspective(img, matrix, (self.output_img_shape_x, self.output_img_shape_y))
        return output


def grass_filter(original_image):
    img = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 0, 40])
    upper = np.array([255, 140, 210])
    mask = cv2.inRange(img, lower, upper)
    mask = 255 - mask
    return mask


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def main(args=None):
    rclpy.init(args=args)
    lane_map_publisher = LaneMapPublisher()
    rclpy.spin(lane_map_publisher)
    lane_map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
