import dearpygui.dearpygui as dpg
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time
import threading
import copy
import numpy as np
import argparse
from hday_motion_planner_msgs.srv import Move
from hday_motion_planner_msgs.msg import CartesianState

from scipy.spatial.transform import Rotation as sciR


parser = argparse.ArgumentParser()
parser.add_argument("--num_joints", type=int, default=7)
parser.add_argument("--pub_topic_name", type=str, default="/arm/target_joint_state")
parser.add_argument("--sub_topic_name_joint", type=str, default="/arm/joint_state")
parser.add_argument(
    "--sub_topic_name_cart",
    type=str,
    default="/hday/engine/motion_planner/end_effector_poses",
)
parser.add_argument("--len_histories", type=int, default=1000)
parser.add_argument("--end_effector_link", type=str, default=None)
args = parser.parse_args()

publish_interval = {"hz": 100}


class JointInterfaceNode(Node):
    def __init__(
        self,
        joint_vel,
        joint_effort,
        publish_topic_name,
        subscribe_topic_name_joint,
        subscribe_topic_name_cart,
    ):
        super().__init__("joint_interface_gui")
        self.joint_vel = joint_vel
        self.joint_effort = joint_effort

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            JointState, publish_topic_name, self.qos_profile
        )
        self.joint_subscription = self.create_subscription(
            JointState,
            subscribe_topic_name_joint,
            self.joint_state_callback,
            self.qos_profile,
        )
        self.cart_subscription = self.create_subscription(
            CartesianState,
            subscribe_topic_name_cart,
            self.cartesian_state_callback,
            self.qos_profile,
        )

        self.motion_planner_client = self.create_client(
            Move, "/hday/engine/motion_planner/move"
        )
        self.move_srv = Move.Request()


        self.joint_values = {}
        self.interpolated_joint_values = {}
        self.joint_efforts = {}
        self.actual_joint_values = {}
        self.joint_limits = {}
        self.joint_names = []

        self.cart_values = {}
        self.ini = True

        self.sin_mag = 0.0
        self.sin_omega = 0.0
        self.sin_time = {}
        self.sin_ref_val = {}
        self.sin_mode = False
        self.selected_sin_joint = {}
        self.mp_future = None

    def publish_joint_command(self):
        if not self.ini:
            self.interpolate()
            if self.sin_mode:
                for n in self.joint_names:
                    if n in self.selected_sin_joint.values():
                        self.joint_values[n] = self.sin_ref_val[n] + self.sin_mag * np.sin(
                            self.sin_omega * (time.time() - self.sin_time[n])
                        )
                        self.interpolated_joint_values[n] = copy.deepcopy(
                            self.joint_values[n]
                        )

                        dpg.set_value(f"{n}_slider", self.joint_values[n])
                        dpg.set_value(f"{n}_input", self.joint_values[n])

            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [self.interpolated_joint_values[n] for n in self.joint_names]
            msg.velocity = [0.0 for n in self.joint_names]
            msg.effort = [self.joint_efforts[n] for n in self.joint_names]

            self.publisher.publish(msg)

    def send_mp_command(self, joint_target=None, cartesian_target=None, mode="joint"):
        self.move_srv.stamp = self.get_clock().now().to_msg()

        if mode == "joint":
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.move_srv.stamp
            msg.name = self.joint_names
            msg.position = joint_target.tolist()
            msg.velocity = [0.0 for i in range(joint_target.shape[0])]
            msg.effort = [0.0 for i in range(joint_target.shape[0])]
            self.move_srv.joint_target = msg
            self.move_srv.cartesian_target = CartesianState()
        elif mode == "cartesian":
            self.move_srv.joint_target = JointState()
            self.move_srv.cartesian_target = cartesian_target

        self.mp_future = self.motion_planner_client.call_async(self.move_srv)

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.actual_joint_values[name] = msg.position[i]

        if self.ini:
            for i, name in enumerate(msg.name):
                self.joint_names.append(name)
                self.joint_values[name] =  msg.position[i]
                self.interpolated_joint_values[name] = msg.position[i]
                self.joint_efforts[name] = self.joint_effort
                self.joint_limits[name] = (-3.14, 3.14)
            self.ini = False

    def cartesian_state_callback(self, msg):
        for name, base_frame, pose in zip(msg.name, msg.base_frame, msg.pose):
            self.cart_values[name] = {
                "name": name,
                "base_frame": base_frame,
                "pose": pose,
            }

    def interpolate(self):
        for n in self.joint_names:
            if self.joint_values[n] > self.interpolated_joint_values[n]:
                self.interpolated_joint_values[n] += (
                    self.joint_vel / publish_interval["hz"]
                )
                if self.interpolated_joint_values[n] > self.joint_values[n]:
                    self.interpolated_joint_values[n] = self.joint_values[n]
            elif self.joint_values[n] < self.interpolated_joint_values[n]:
                self.interpolated_joint_values[n] -= (
                    self.joint_vel / publish_interval["hz"]
                )
                if self.interpolated_joint_values[n] < self.joint_values[n]:
                    self.interpolated_joint_values[n] = self.joint_values[n]


class RobotJog:
    def __init__(
        self,
        ros_node,
        joint_effort,
        len_joint_histories,
        keyboard_delta_val,
    ):
        self.ros_node = ros_node
        self.joint_effort = joint_effort
        self.len_joint_histories = len_joint_histories
        self.joint_histories = {}
        self.actual_histories = {}
        self.keyboard_delta_val = keyboard_delta_val

        self.selected_joint = {"name": None}
        self.key_hold_start = {}

        self.publishing_enabled = {"active": False}
        self.autofit_enabled = {"active": True}
        self.last_publish_time = {"t": 0.0}

        self.highlight_theme = self.create_highlight_theme()
        self.publish_timestamps = []

        self.cart_position_delta = 0.0

    def select_joint_callback(self, sender):
        if self.selected_joint["name"] == sender:
            # Deselect if already selected
            dpg.bind_item_theme(sender, 0)
            self.selected_joint["name"] = None
            self.ros_node.sin_mode = False
        else:
            for j in self.ros_node.joint_names:
                dpg.bind_item_theme(j, 0)
                dpg.bind_item_theme(f"Sin({j})", 0)
            dpg.bind_item_theme(sender, self.highlight_theme)
            self.selected_joint["name"] = sender
            self.ros_node.sin_mode = False

    def select_sin_mode_callback(self, sender):
        if sender in self.ros_node.selected_sin_joint.keys():
            # Deselect if already selected
            dpg.bind_item_theme(sender, 0)
            self.ros_node.selected_sin_joint.pop(sender, None)
            if len(self.ros_node.selected_sin_joint) == 0:
                self.ros_node.sin_mode = False
        else:
            for j in self.ros_node.joint_names:
                dpg.bind_item_theme(j, 0)
            dpg.bind_item_theme(sender, self.highlight_theme)

            for name in self.ros_node.joint_names:
                if name in sender:
                    self.ros_node.selected_sin_joint[sender] = name
                    self.ros_node.sin_time[name] = time.time()
                    self.ros_node.sin_ref_val[name] = self.ros_node.interpolated_joint_values[name]
                    self.ros_node.sin_mode = True

    def keyboard_callback(self, sender, app_data):
        if not isinstance(app_data, (list, tuple)) or len(app_data) != 2:
            return

        key_code, is_down = app_data

        if is_down:
            now = self.ros_node.get_clock().now().nanoseconds / 1e9
            last_time = self.key_hold_start.get(key_code, 0)
            elapsed = now - last_time

            # Only allow once immediately or after hold threshold
            if elapsed < 0.1 and last_time != 0:
                return
            self.key_hold_start[key_code] = now

            if key_code == 524:  ## space
                self.start_publishing()
            elif key_code == 526:  ## exc
                self.stop_publishing()

            joint = self.selected_joint["name"]
            if joint is None:
                return

            if key_code == dpg.mvKey_Up:
                delta = self.keyboard_delta_val
            elif key_code == dpg.mvKey_Down:
                delta = -self.keyboard_delta_val
            else:
                return

            val = self.ros_node.joint_values[joint] + delta
            min_val, max_val = self.ros_node.joint_limits[joint]
            val = max(min_val, min(max_val, val))
            self.ros_node.joint_values[joint] = val

            dpg.set_value(f"{joint}_slider", val)
            dpg.set_value(f"{joint}_input", val)

    def slider_callback(self, sender):
        joint = sender.replace("_slider", "")
        val = dpg.get_value(sender)
        self.ros_node.joint_values[joint] = val
        dpg.set_value(f"{joint}_slider", val)
        dpg.set_value(f"{joint}_input", val)

    def input_callback(self, sender):
        joint = sender.replace("_input", "")
        val = dpg.get_value(sender)
        min_val, max_val = self.ros_node.joint_limits[joint]
        val = max(min_val, min(max_val, val))
        self.ros_node.joint_values[joint] = val
        dpg.set_value(f"{joint}_slider", val)
        dpg.set_value(f"{joint}_input", val)

    def sin_mag_callback(self, sender):
        val = dpg.get_value(sender)
        if not self.ros_node.sin_mode:
            dpg.set_value(f"sin_mag", val)
            self.ros_node.sin_mag = val
        else:
            dpg.set_value(f"sin_mag", self.ros_node.sin_mag)

    def sin_omega_callback(self, sender):
        val = dpg.get_value(sender)
        if not self.ros_node.sin_mode:
            dpg.set_value(f"sin_omega", val)
            self.ros_node.sin_omega = val
        else:
            dpg.set_value(f"sin_omega", self.ros_node.sin_omega)

    def cart_position_delta_callback(self, sender):
        val = dpg.get_value(sender)
        dpg.set_value(f"cart_position_delta", val)
        self.cart_position_delta = val

    def cart_orientation_delta_callback(self, sender):
        val = dpg.get_value(sender)
        dpg.set_value(f"cart_orientation_delta", val)
        self.cart_orientation_delta = val

    def update_joint_effort(self, sender):
        joint = sender.replace("_effort", "")
        self.ros_node.joint_efforts[joint] = dpg.get_value(sender)

    def set_all_efforts(self, sender):
        val = dpg.get_value("Effort All")
        for joint in self.ros_node.joint_names:
            self.ros_node.joint_efforts[joint] = val
            dpg.set_value(f"{joint}_effort", val)

    def update_publish_interval(self, sender):
        val = dpg.get_value(sender)
        publish_interval["hz"] = min(100, val)

    def start_publishing(self):
        self.publishing_enabled["active"] = True
        print("Publishing started.")
        dpg.bind_item_theme("pub_start_button", self.highlight_theme)
        dpg.bind_item_theme("pub_stop_button", 0)

    def stop_publishing(self):
        self.publishing_enabled["active"] = False
        print("Publishing stopped.")
        dpg.bind_item_theme("pub_stop_button", self.highlight_theme)
        dpg.bind_item_theme("pub_start_button", 0)

    def set_zero(self):
        if self.publishing_enabled["active"]:
            self.stop_publishing()
            time.sleep(0.5)
        joint_target = np.zeros(len(self.ros_node.joint_names))
        self.ros_node.send_mp_command(joint_target=joint_target, mode="joint")
        print("Set ZERO")

    def cart_set(self, dir):
        if self.publishing_enabled["active"]:
            self.stop_publishing()
            time.sleep(0.5)

        if len(self.ros_node.cart_values) > 0:
            if args.end_effector_link in self.ros_node.cart_values.keys():
                msg = CartesianState()
                msg.name.append(
                    self.ros_node.cart_values[args.end_effector_link]["name"]
                )
                msg.base_frame.append(
                    self.ros_node.cart_values[args.end_effector_link]["base_frame"]
                )
                pose = self.ros_node.cart_values[args.end_effector_link]["pose"]
                if dir == "+x":
                    pose.position.x += self.cart_position_delta
                elif dir == "-x":
                    pose.position.x -= self.cart_position_delta
                elif dir == "+y":
                    pose.position.y += self.cart_position_delta
                elif dir == "-y":
                    pose.position.y -= self.cart_position_delta
                elif dir == "+z":
                    pose.position.z += self.cart_position_delta
                elif dir == "-z":
                    pose.position.z -= self.cart_position_delta

                quat = np.array(
                    [
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                )
                if dir == "+roll":
                    quat = (
                        sciR.from_quat(quat)
                        * sciR.from_euler("x", self.cart_orientation_delta)
                    ).as_quat()
                elif dir == "-roll":
                    quat = (
                        sciR.from_quat(quat)
                        * sciR.from_euler("x", -self.cart_orientation_delta)
                    ).as_quat()
                elif dir == "+pitch":
                    quat = (
                        sciR.from_quat(quat)
                        * sciR.from_euler("y", self.cart_orientation_delta)
                    ).as_quat()
                elif dir == "-pitch":
                    quat = (
                        sciR.from_quat(quat)
                        * sciR.from_euler("y", -self.cart_orientation_delta)
                    ).as_quat()
                elif dir == "+yaw":
                    quat = (
                        sciR.from_quat(quat)
                        * sciR.from_euler("z", self.cart_orientation_delta)
                    ).as_quat()
                elif dir == "-yaw":
                    quat = (
                        sciR.from_quat(quat)
                        * sciR.from_euler("z", -self.cart_orientation_delta)
                    ).as_quat()

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                msg.pose.append(pose)
                self.ros_node.send_mp_command(cartesian_target=msg, mode="cartesian")
            else:
                print("End-Effector link does not exist")
        else:
            print("Cannot subscribe cartesian pose")

    def plus_x(self):
        self.cart_set("+x")

    def minus_x(self):
        self.cart_set("-x")

    def plus_y(self):
        self.cart_set("+y")

    def minus_y(self):
        self.cart_set("-y")

    def plus_z(self):
        self.cart_set("+z")

    def minus_z(self):
        self.cart_set("-z")

    def plus_roll(self):
        self.cart_set("+roll")

    def minus_roll(self):
        self.cart_set("-roll")

    def plus_pitch(self):
        self.cart_set("+pitch")

    def minus_pitch(self):
        self.cart_set("-pitch")

    def plus_yaw(self):
        self.cart_set("+yaw")

    def minus_yaw(self):
        self.cart_set("-yaw")

    def auto_fit(self):
        if self.autofit_enabled["active"]:
            self.autofit_enabled["active"] = False
            dpg.bind_item_theme("Autofit", 0)
        else:
            self.autofit_enabled["active"] = True
            dpg.bind_item_theme("Autofit", self.highlight_theme)

    def create_line_thickness_theme(self, thickness=5.0):
        with dpg.theme() as theme:
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_style(
                    dpg.mvPlotStyleVar_LineWeight,
                    thickness,
                    category=dpg.mvThemeCat_Plots,
                )

        return theme

    def create_highlight_theme(self):
        with dpg.theme() as theme_id:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (0, 150, 250, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (0, 200, 255, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (0, 100, 200, 255))
        return theme_id

    def clear_last_key(self):
        self.key_hold_start.clear()

    def on_exit(self):
        rclpy.shutdown()
        dpg.stop_dearpygui()

    def update_all(self):
        if self.ros_node.mp_future is not None:
            if not self.publishing_enabled["active"] and self.ros_node.mp_future.done():
                self.ros_node.mp_future = None
                time.sleep(0.5)

                self.ros_node.joint_values = copy.deepcopy(
                    self.ros_node.actual_joint_values
                )
                self.ros_node.interpolated_joint_values = copy.deepcopy(
                    self.ros_node.actual_joint_values
                )

                for joint in self.ros_node.joint_names:
                    dpg.set_value(f"{joint}_slider", self.ros_node.joint_values[joint])
                    dpg.set_value(f"{joint}_input", self.ros_node.joint_values[joint])
                self.start_publishing()

        now = self.ros_node.get_clock().now().nanoseconds / 1e9
        if self.publishing_enabled["active"] and (
            now - self.last_publish_time["t"] >= 1 / publish_interval["hz"]
        ):

            for joint in self.ros_node.joint_names:
                self.joint_histories[joint].append(
                    self.ros_node.interpolated_joint_values[joint]
                )
                if len(self.joint_histories[joint]) > self.len_joint_histories:
                    self.joint_histories[joint].pop(0)
                dpg.set_value(
                    f"{joint}_line_cmd",
                    [
                        list(range(len(self.joint_histories[joint]))),
                        self.joint_histories[joint],
                    ],
                )

                self.actual_histories[joint].append(
                    self.ros_node.actual_joint_values[joint]
                )
                if len(self.actual_histories[joint]) > self.len_joint_histories:
                    self.actual_histories[joint].pop(0)
                dpg.set_value(
                    f"{joint}_line_actual",
                    [
                        list(range(len(self.actual_histories[joint]))),
                        self.actual_histories[joint],
                    ],
                )
                dpg.set_value(
                    f"{joint}_actual_text",
                    f"Actual {joint}: {self.ros_node.actual_joint_values[joint]:.3f}",
                )
                dpg.set_value(
                    f"{joint}_interpolated_input",
                    f"{self.ros_node.interpolated_joint_values[joint]:.3f}",
                )

                if self.autofit_enabled["active"]:
                    dpg.fit_axis_data(f"{joint}_yaxis")
                    min_val = (
                        min(
                            min(self.actual_histories[joint]),
                            min(self.joint_histories[joint]),
                        )
                        - 0.05
                    )
                    max_val = (
                        max(
                            max(self.actual_histories[joint]),
                            max(self.joint_histories[joint]),
                        )
                        + 0.05
                    )
                    dpg.set_axis_limits(f"{joint}_yaxis", min_val, max_val)

                    dpg.fit_axis_data(f"{joint}_xaxis")
                    dpg.set_axis_limits(f"{joint}_xaxis", 0, self.len_joint_histories)
                else:
                    dpg.set_axis_limits_auto(f"{joint}_xaxis")
                    dpg.set_axis_limits_auto(f"{joint}_yaxis")

            # rclpy.spin_once(ros_node, timeout_sec=0)
            # now = self.ros_node.get_clock().now().nanoseconds / 1e9
            # if self.publishing_enabled["active"] and (
            #     now - self.last_publish_time["t"] >= 1 / publish_interval["hz"]
            # ):
            self.ros_node.publish_joint_command()
            self.last_publish_time["t"] = now
            self.publish_timestamps.append(now)
            if len(self.publish_timestamps) > 10:
                self.publish_timestamps.pop(0)

            if len(self.publish_timestamps) >= 2 and dpg.does_item_exist(
                "Actual Publish Hz"
            ):
                intervals = [
                    t2 - t1
                    for t1, t2 in zip(
                        self.publish_timestamps[:-1], self.publish_timestamps[1:]
                    )
                ]
                avg_interval = sum(intervals) / len(intervals)
                actual_hz = 1.0 / avg_interval if avg_interval > 0 else 0.0
                dpg.set_value("Actual Publish Hz", f"Actual: {actual_hz:.2f} Hz")

            if not self.publishing_enabled["active"]:
                actual_hz = 0.0
                dpg.set_value("Actual Publish Hz", f"Actual: {actual_hz:.2f} Hz")

    def setup_ui(self):
        def ros_spin_loop():
            while rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.01)

        threading.Thread(target=ros_spin_loop, daemon=True).start()

        while self.ros_node.ini:
            print("Cannot subscribe robot joint state")
            time.sleep(0.5)

        self.joint_histories = {
            name: [0.0] * self.len_joint_histories for name in self.ros_node.joint_names
        }
        self.actual_histories = {
            name: [0.0] * self.len_joint_histories for name in self.ros_node.joint_names
        }

        with dpg.window(label="Joint Control Panel", tag="main_window"):
            with dpg.group(horizontal=True):
                with dpg.child_window(width=600):

                    with dpg.group(horizontal=True):
                        with dpg.group():
                            dpg.add_text("Select Joint", tag="text_joint_select")
                            for name in self.ros_node.joint_names:
                                dpg.add_button(
                                    label=name,
                                    tag=name,
                                    callback=self.select_joint_callback,
                                )

                        dpg.add_spacer(width=20)
                        with dpg.group():
                            dpg.add_text("Sinusoidal Motion", tag="text_sin_motion")
                            for name in self.ros_node.joint_names:
                                dpg.add_button(
                                    label=f"Sin({name})",
                                    tag=f"Sin({name})",
                                    callback=self.select_sin_mode_callback,
                                )

                        dpg.add_spacer(width=10)
                        with dpg.group():
                            dpg.add_spacer(height=70)
                            with dpg.group(horizontal=True):
                                dpg.add_text(
                                    f"Sin Magnitude (rad):",
                                )
                                dpg.add_input_float(
                                    tag="sin_mag",
                                    default_value=0.0,
                                    callback=self.sin_mag_callback,
                                    width=100,
                                    on_enter=True,
                                )
                            with dpg.group(horizontal=True):
                                dpg.add_text(
                                    f"Sin Omega (rad/s):",
                                )
                                dpg.add_input_float(
                                    tag="sin_omega",
                                    default_value=0.0,
                                    callback=self.sin_omega_callback,
                                    width=100,
                                    on_enter=True,
                                )

                    dpg.add_spacer(height=10)
                    dpg.add_text("Joint Commands", tag="text_joint_commands")
                    for name in self.ros_node.joint_names:
                        min_val, max_val = self.ros_node.joint_limits[name]
                        with dpg.group(horizontal=True):
                            dpg.add_text(f"{name}", bullet=True)
                            dpg.add_slider_float(
                                tag=f"{name}_slider",
                                # label=f"{name} ({min_val:.2f} ~ {max_val:.2f})",
                                # label=f"{name}",
                                default_value=self.ros_node.actual_joint_values[name],
                                min_value=min_val,
                                max_value=max_val,
                                callback=self.slider_callback,
                                user_data=name,
                                width=300,
                                format="%.3f",
                                clamped=True,
                            )
                            dpg.add_input_float(
                                tag=f"{name}_input",
                                default_value=self.ros_node.actual_joint_values[name],
                                callback=self.input_callback,
                                width=100,
                                on_enter=True,
                            )
                            dpg.add_text(
                                tag=f"{name}_interpolated_input",
                                default_value=self.ros_node.actual_joint_values[name],
                            )

                    dpg.add_spacer(height=10)
                    dpg.add_text("Joint States", tag="text_joint_states")
                    for joint in self.ros_node.joint_names:
                        dpg.add_text(
                            f"Actual: 0.000", tag=f"{joint}_actual_text", bullet=True
                        )

                    dpg.add_spacer(height=10)
                    dpg.add_text("Individual Effort Settings", tag="text_effort")
                    for name in self.ros_node.joint_names:
                        dpg.add_input_float(
                            tag=f"{name}_effort",
                            label=f"{name} Effort",
                            default_value=self.joint_effort,
                            callback=self.update_joint_effort,
                            width=350,
                            on_enter=True,
                        )
                    dpg.add_input_float(
                        tag="Effort All",
                        label="Effort All",
                        default_value=self.joint_effort,
                        callback=self.set_all_efforts,
                        width=350,
                        on_enter=True,
                    )

                    dpg.add_spacer(height=10)
                    dpg.add_text("Publishing Control", tag="text_control")
                    with dpg.group(horizontal=True):
                        dpg.add_button(
                            label="Start (SPACE)",
                            callback=self.start_publishing,
                            tag="pub_start_button",
                        )
                        dpg.add_button(
                            label="Stop (ESC)",
                            callback=self.stop_publishing,
                            tag="pub_stop_button",
                        )
                    dpg.bind_item_theme("pub_stop_button", self.highlight_theme)

                    with dpg.group(horizontal=False):
                        dpg.add_input_float(
                            tag="Publish Interval (Hz)",
                            default_value=publish_interval["hz"],
                            callback=self.update_publish_interval,
                            on_enter=True,
                            step=10,
                        )
                        dpg.add_text("", tag="Actual Publish Hz")

                    # dpg.add_spacer(height=10)
                    dpg.add_text("Motion Planner Command", tag="text_mp_command")
                    dpg.add_button(
                        label="ZERO",
                        callback=self.set_zero,
                        tag="set_zero_button",
                        width=60,
                        height=40,
                    )

                    if args.end_effector_link is not None:
                        with dpg.group(horizontal=True):
                            dpg.add_button(
                                label="+x",
                                callback=self.plus_x,
                                tag="plus_x_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="-x",
                                callback=self.minus_x,
                                tag="minus_x_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="+y",
                                callback=self.plus_y,
                                tag="plus_y_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="-y",
                                callback=self.minus_y,
                                tag="minus_y_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="+z",
                                callback=self.plus_z,
                                tag="plus_z_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="-z",
                                callback=self.minus_z,
                                tag="minus_z_button",
                                width=60,
                                height=40,
                            )

                        with dpg.group(horizontal=True):
                            dpg.add_button(
                                label="+roll",
                                callback=self.plus_roll,
                                tag="plus_roll_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="-roll",
                                callback=self.minus_roll,
                                tag="minus_roll_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="+pitch",
                                callback=self.plus_pitch,
                                tag="plus_pitch_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="-pitch",
                                callback=self.minus_pitch,
                                tag="minus_pitch_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="+yaw",
                                callback=self.plus_yaw,
                                tag="plus_yaw_button",
                                width=60,
                                height=40,
                            )
                            dpg.add_button(
                                label="-yaw",
                                callback=self.minus_yaw,
                                tag="minus_yaw_button",
                                width=60,
                                height=40,
                            )

                    if args.end_effector_link is not None:
                        with dpg.group(horizontal=True):
                            dpg.add_text(
                                f"Cartesian Position Delta (m):",
                            )
                            dpg.add_input_float(
                                tag="cart_position_delta",
                                default_value=0.0,
                                callback=self.cart_position_delta_callback,
                                width=100,
                                step=0.01,
                                on_enter=True,
                            )

                        with dpg.group(horizontal=True):
                            dpg.add_text(
                                f"Cartesian Orientation Delta (rad):",
                            )
                            dpg.add_input_float(
                                tag="cart_orientation_delta",
                                default_value=0.0,
                                callback=self.cart_orientation_delta_callback,
                                width=100,
                                step=0.01,
                                on_enter=True,
                            )

                with dpg.child_window(width=600, tag="second_child_window"):
                    dpg.add_text("Realtime Plot", tag="text_graph")
                    dpg.add_button(
                        label="AutoFit", callback=self.auto_fit, tag="Autofit"
                    )
                    dpg.bind_item_theme("Autofit", self.highlight_theme)
                    for name in self.ros_node.joint_names:
                        min_val, max_val = self.ros_node.joint_limits[name]
                        with dpg.plot(
                            label=f"{name}_plot",
                            height=160,
                            width=-1,
                            tag=f"{name}_plot",
                        ):
                            dpg.add_plot_legend()
                            # dpg.add_plot_axis(dpg.mvXAxis, label="", auto_fit=True)
                            # with dpg.plot_axis(
                            #     dpg.mvYAxis, label="", auto_fit=True
                            # ) as y_axis:
                            dpg.add_plot_axis(
                                dpg.mvXAxis, label="", tag=f"{name}_xaxis"
                            )
                            with dpg.plot_axis(
                                dpg.mvYAxis, label="", tag=f"{name}_yaxis"
                            ) as y_axis:
                                # dpg.set_axis_limits(y_axis, min_val, max_val)
                                dpg.add_line_series(
                                    list(range(self.len_joint_histories)),
                                    self.joint_histories[name],
                                    label="command",
                                    tag=f"{name}_line_cmd",
                                )
                                dpg.add_line_series(
                                    list(range(self.len_joint_histories)),
                                    self.actual_histories[name],
                                    label="actual",
                                    tag=f"{name}_line_actual",
                                )
                                line_thickness_theme = self.create_line_thickness_theme(
                                    3
                                )
                                dpg.bind_item_theme(
                                    f"{name}_line_cmd", line_thickness_theme
                                )
                                dpg.bind_item_theme(
                                    f"{name}_line_actual", line_thickness_theme
                                )

        dpg.set_exit_callback(self.on_exit)
        dpg.create_viewport(
            title="Joint Controller", width=1220, height=1200, resizable=True
        )
        dpg.setup_dearpygui()
        dpg.set_primary_window("main_window", True)
        with dpg.handler_registry():
            dpg.add_key_down_handler(callback=self.keyboard_callback)
            dpg.add_key_release_handler(callback=lambda s, a: self.clear_last_key())
        dpg.show_viewport()

        def run_update_loop():
            while dpg.is_dearpygui_running():
                self.update_all()
                # time.sleep(1.0 / 100)

        threading.Thread(target=run_update_loop, daemon=True).start()
        dpg.start_dearpygui()
        dpg.destroy_context()


def main():
    joint_effort = 0.8
    publish_topic_name = args.pub_topic_name
    subscribe_topic_name_joint = args.sub_topic_name_joint
    subscribe_topic_name_cart = args.sub_topic_name_cart
    # publish_topic_name = "/arm/joint_states"
    # subscribe_top
    # ic_name = "/arm/joint_states_temp"
    joint_vel = 0.5 ## rad/s
    keyboard_delta_val = 0.05  ## rad
    len_joint_histories = args.len_histories

    rclpy.init()
    ros_node = JointInterfaceNode(
        joint_vel,
        joint_effort,
        publish_topic_name,
        subscribe_topic_name_joint,
        subscribe_topic_name_cart,
    )
    dpg.create_context()
    robot_jog = RobotJog(
        ros_node,
        joint_effort,
        len_joint_histories,
        keyboard_delta_val,
    )
    robot_jog.setup_ui()


if __name__ == "__main__":
    main()
