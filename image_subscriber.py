import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber_with_filters")
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.listener_callback, 10
        )
        self.bridge = CvBridge()
        self.current_filter = 0
        self.filters = [
            self.no_filter,
            self.gray_filter,
            self.blur_filter,
            self.threshold_filter,
            self.canny_filter,
            self.sobel_filter,
            self.sketch_filter,
            self.cartoon_filter,
        ]
        self.filter_names = [
            "No Filter",
            "Gray Filter",
            "Blur Filter",
            "Threshold Filter",
            "Canny Filter",
            "Sobel Filter",
            "Sketch Filter",
            "Cartoon Filter",
        ]
        self.get_logger().info("Image subscriber with filters initialized.")

    def no_filter(self, img):
        return img

    def gray_filter(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def blur_filter(self, img):
        return cv2.GaussianBlur(img, (15, 15), 0)

    def threshold_filter(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
        return thresh

    def canny_filter(self, img):
        return cv2.Canny(img, 100, 200)

    def sobel_filter(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=5)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=5)
        sobel = cv2.magnitude(sobel_x, sobel_y)
        sobel = cv2.convertScaleAbs(sobel)
        return sobel

    def sketch_filter(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        _, sketch = cv2.threshold(edges, 50, 255, cv2.THRESH_BINARY_INV)
        return sketch

    def cartoon_filter(self, img):
        # Redução de cores
        color = cv2.bilateralFilter(img, d=9, sigmaColor=75, sigmaSpace=75)

        # Bordas em preto e branco
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(gray, 7)
        edges = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,
            blockSize=9,
            C=2,
        )

        # Combina contornos com imagem colorida
        cartoon = cv2.bitwise_and(color, color, mask=edges)
        return cartoon

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_image = cv2.resize(cv_image, dsize=(640, 480))
            filtered_image = self.filters[self.current_filter](cv_image)

            # Converte para BGR se necessário
            if len(filtered_image.shape) == 2:
                filtered_image = cv2.cvtColor(filtered_image, cv2.COLOR_GRAY2BGR)

            self.show_with_label(filtered_image, self.filter_names[self.current_filter])
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord("q"):
                raise KeyboardInterrupt()
            elif key == 81:  # ← Left arrow
                self.current_filter = (self.current_filter - 1) % len(self.filters)
                self.get_logger().info(f"Switched to {self.filter_names[self.current_filter]}")
            elif key == 83:  # → Right arrow
                self.current_filter = (self.current_filter + 1) % len(self.filters)
                self.get_logger().info(f"Switched to {self.filter_names[self.current_filter]}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def show_with_label(self, img, filter_name):
        labeled_img = img.copy()
        if len(labeled_img.shape) == 2:
            labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_GRAY2BGR)

        cv2.putText(
            labeled_img,
            f"{filter_name}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )

        cv2.imshow("Image with Filter", labeled_img)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
