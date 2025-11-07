#!/usr/bin/env python3
import os
import yaml
import random
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import py_trees
import py_trees_ros
from py_trees.common import OneShotPolicy
from ament_index_python.packages import get_package_share_directory
from py_trees.blackboard import Blackboard
from std_msgs.msg import Int32

from tb_behaviors.navigation import GoToPose, GetLocationFromQueue
from tb_behaviors.vision import LookForObject

default_location_file = os.path.join(
    get_package_share_directory("tb_worlds"), "maps", "sim_house_locations.yaml"
)


class BatteryLow(py_trees.behaviour.Behaviour):
    def __init__(self, threshold=35):
        super().__init__("BatteryLow")
        self.threshold = threshold
        self.bb = Blackboard()

    def update(self):
        val = self.bb.get("battery_pct")
        if val is None:
            val = 100
        return (py_trees.common.Status.SUCCESS
                if int(val) < self.threshold
                else py_trees.common.Status.FAILURE)


class Recharge(py_trees.behaviour.Behaviour):
    """
    Non-blocking 5-second recharge behavior:
    - Uses Blackboard keys so the timer is not reset on repeated initialise() calls.
      Keys used: 'recharge_active' (bool), 'recharge_start_ns' (int nanoseconds)
    """
    def __init__(self, node):
        super().__init__("Recharge")
        self.node = node
        self.bb = Blackboard()

    def initialise(self):
        active = self.bb.get('recharge_active')
        if active:
            return
        now = self.node.get_clock().now()
        self.bb.set('recharge_active', True)
        self.bb.set('recharge_start_ns', int(now.nanoseconds))
        self.node.get_logger().info("Starting recharge (5s)...")

    def update(self):
        active = self.bb.get('recharge_active')
        if not active:
            return py_trees.common.Status.RUNNING

        start_ns = self.bb.get('recharge_start_ns')
        if start_ns is None:
            start_ns = int(self.node.get_clock().now().nanoseconds)
            self.bb.set('recharge_start_ns', start_ns)

        now = self.node.get_clock().now()
        elapsed_ns = int(now.nanoseconds) - int(start_ns)
        if elapsed_ns >= int(5 * 1e9):  # 5 seconds
            self.bb.set('recharge_active', False)
            self.bb.set('recharge_start_ns', None)

            # set battery to 100 everywhere
            self.bb.set("battery_pct", 100)
            try:
                self.node.battery_pct = 100
            except Exception:
                pass
            try:
                msg = Int32()
                msg.data = 100
                self.node.batt_pub.publish(msg)
            except Exception:
                pass
            self.node.get_logger().info("Recharge complete -> 100%")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class MissionComplete(py_trees.behaviour.Behaviour):
    """Returns SUCCESS when loc_list is empty (mission finished)."""
    def __init__(self):
        super().__init__("MissionComplete")
        self.bb = Blackboard()

    def update(self):
        locs = self.bb.get("loc_list")
        if not locs:
            self.logger.info("No blocks left - Mission Complete")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class AutonomyBehavior(Node):
    def __init__(self):
        super().__init__("autonomy_node_python")

        self.declare_parameter("enable_vision", value=True)
        self.declare_parameter("target_color", value="blue")

        location_file = default_location_file
        with open(location_file, "r") as f:
            self.locations = yaml.load(f, Loader=yaml.FullLoader)

        self.loc_list = list(self.locations.keys())

        # charger coordinates - per your request z=0.5
        self.charging_pose = (2.8, 3.5, 0.5)

        # node-authoritative battery variable
        self.battery_pct = 100
        self.batt_pub = self.create_publisher(Int32, "battery_pct", 10)

        # Drain settings
        self._per_second_drain_normal = 2   # normal drain per second (%)
        self._per_second_drain_power_save = 1  # drain when in power-save threshold
        self._power_save_threshold = 30     # % at which we enter power-save mode
        self._per_second_drain = self._per_second_drain_normal

        # detection drain (when a block is detected)
        self._detect_drain = 15  # % drained per detection

        # per-second timer
        self.create_timer(1.0, self._battery_timer)

        bb = Blackboard()
        bb.set("loc_list", list(self.loc_list))
        bb.set("battery_pct", int(self.battery_pct))
        # init recharge flags
        bb.set('recharge_active', False)
        bb.set('recharge_start_ns', None)

        self.tree = self.build_tree()
        self.get_logger().info(f"Autonomy node ready (charger at {self.charging_pose})")
        self.get_logger().info(f"Using location file: {location_file}")
        self.get_logger().info("Looking for blocks (red, green, blue)...")

    def _battery_timer(self):
        bb = Blackboard()
        recharge_active = bb.get('recharge_active')

        # If charging, do not drain
        if not recharge_active:
            # choose drain amount (power-save when low battery)
            if self.battery_pct <= self._power_save_threshold:
                drain_amt = self._per_second_drain_power_save
            else:
                drain_amt = self._per_second_drain_normal

            # drain if battery > 0
            if self.battery_pct > 0:
                # subtract drain_amt (even if battery is 100 -> now it will start dropping)
                self.battery_pct = max(0, int(self.battery_pct - drain_amt))

        # publish and set on blackboard
        msg = Int32()
        msg.data = int(self.battery_pct)
        try:
            self.batt_pub.publish(msg)
        except Exception:
            pass
        bb.set("battery_pct", int(self.battery_pct))

        # print battery state every second
        self.get_logger().info(f"Battery: {int(self.battery_pct)}%")

    def build_tree(self):
        top = py_trees.composites.Selector("TopSelector", memory=False)

        mission_complete = MissionComplete()

        charge_seq = py_trees.composites.Sequence("Charging", memory=False)
        charge_seq.add_children([
            BatteryLow(30),
            GoToPose("GoToCharger", self.charging_pose, self),
            Recharge(self)
        ])

        mission_seq = py_trees.composites.Sequence("SearchBlocks", memory=True)
        mission_seq.add_child(GetLocationFromQueue("get_next", self.locations))
        mission_seq.add_child(GoToPose("go_to", None, self))

        # Try 3 colors one-by-one
        detect_any = py_trees.composites.Selector("DetectAnyColor", memory=False)
        detect_any.add_children([
            LookForObject("Find_blue", "blue", self, save_folder="/home/rohith/bt_images"),
            LookForObject("Find_red", "red", self, save_folder="/home/rohith/bt_images"),
            LookForObject("Find_green", "green", self, save_folder="/home/rohith/bt_images"),
            py_trees.behaviours.Success("NoColorFound")
        ])
        mission_seq.add_child(detect_any)

        # top ordering: if mission complete -> success; else if battery low -> charge; else search
        top.add_children([mission_complete, charge_seq, mission_seq])

        root = top
        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)
        tree.setup(timeout=15.0, node=self)
        return tree

    def execute(self):
        self.tree.tick_tock(500)
        rclpy.spin(self.tree.node)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = AutonomyBehavior()
    try:
        node.execute()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

