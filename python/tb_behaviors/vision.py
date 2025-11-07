import cv2
import cv_bridge
import os
import py_trees
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

hsv_thresholds = {
    "red": ((160, 220, 0), (180, 255, 255)),
    "green": ((40, 220, 0), (90, 255, 255)),
    "blue": ((100, 220, 0), (150, 255, 255)),
}

class LookForObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, color, node, save_folder="/home/rohith/bt_images"):
        super().__init__(name)
        self.color = color
        self.node = node
        self.save_folder = save_folder
        os.makedirs(self.save_folder, exist_ok=True)

        self.hsv_min, self.hsv_max = hsv_thresholds[color]
        self.bridge = cv_bridge.CvBridge()
        self.latest_img = None

        # subscribe once
        self.sub = self.node.create_subscription(
            Image, "/camera/image_raw", self.img_cb, 10
        )

    def img_cb(self, msg):
        self.latest_img = msg

    def update(self):
        if self.latest_img is None:
            return py_trees.common.Status.RUNNING

        img = self.bridge.imgmsg_to_cv2(self.latest_img, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            filename = f"{self.save_folder}/detected_{self.color}_{int(self.node.get_clock().now().nanoseconds)}.png"
            cv2.imwrite(filename, img)
            self.node.get_logger().info(f"Detected {self.color}, saved: {filename}")

            # reduce battery by detection drain and publish immediately
            bb = py_trees.blackboard.Blackboard()
            current = bb.get("battery_pct")
            if current is None:
                current = getattr(self.node, "battery_pct", 100)
            # detection drain: 30% (as requested)
            new_val = max(0, int(current - 30))
            bb.set("battery_pct", int(new_val))
            # also update node variable and publish so timers see it immediately
            try:
                self.node.battery_pct = int(new_val)
            except Exception:
                pass
            try:
                msg = Int32()
                msg.data = int(new_val)
                self.node.batt_pub.publish(msg)
            except Exception:
                pass

            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(f"No {self.color} detected")
        return py_trees.common.Status.FAILURE

