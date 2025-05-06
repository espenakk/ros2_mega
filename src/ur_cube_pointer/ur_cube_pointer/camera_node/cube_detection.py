# Copyright 2025 Rune Espenakk
#
# Licensed under the MIT License;
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://opensource.org/licenses/MIT
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()

        self.colors = ['red', 'yellow', 'blue']
        self.thresholds = {}

        # Declare parameters for each color
        for color in self.colors:
            self.declare_parameters(
                namespace=f'{color}',
                parameters=[
                    ('hue_low', 0),
                    ('hue_high', 10),
                    ('sat_low', 100),
                    ('sat_high', 255),
                    ('val_low', 100),
                    ('val_high', 255)
                ]
            )

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)

    def image_callback(self, msg):
        self.get_logger().info("Image callback triggered")

        try:

            cv_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2HSV)

            for color in self.colors:
                params = self.get_parameters([
                    f'{color}.hue_low',
                    f'{color}.hue_high',
                    f'{color}.sat_low',
                    f'{color}.sat_high',
                    f'{color}.val_low',
                    f'{color}.val_high',
                ])

                lower_bound = np.array([
                    params[0].value,
                    params[2].value,
                    params[4].value
                ])
                upper_bound = np.array([
                    params[1].value,
                    params[3].value,
                    params[5].value
                ])

                mask = cv2.inRange(hsv, lower_bound, upper_bound)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                positions = []

                for cnt in contours:
                    if cv2.contourArea(cnt) > 500:
                        x, y, w, h = cv2.boundingRect(cnt)
                        cx = x + w // 2
                        cy = y + h // 2
                        positions.append((cx, cy))

                        # Draw rectangle and label
                        cv2.rectangle(cv_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(cv_frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Log the positions
                self.get_logger().info(f'{color.capitalize()} positions: {positions}')

            self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_frame, 'bgr8'))

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    detector = ColorDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


