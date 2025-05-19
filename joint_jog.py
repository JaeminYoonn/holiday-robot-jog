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
from std_msgs.msg import Header
import time
import threading
import copy
import numpy as np

JOINT_NAMES = [f"arm/joint{i+1}" for i in range(7)]
JOINT_LIMITS = {name: (-3.14, 3.14) for name in JOINT_NAMES}
# PUBLISH_TOPIC = "/arm/target_joint_state"
# SUBSCRIBE_TOPIC = "/arm/joint_states"
PUBLISH_TOPIC = "/arm/joint_states"
SUBSCRIBE_TOPIC = "/arm/joint_states_temp"
JOINT_VEL = 0.2  ## rad/s
KEYBOARD_DELTA_VAL = 0.05  ## rad
LEN_JOINT_HISTORIES = 300

joint_histories = {name: [0.0] * LEN_JOINT_HISTORIES for name in JOINT_NAMES}
actual_histories = {name: [0.0] * LEN_JOINT_HISTORIES for name in JOINT_NAMES}
selected_joint = {"name": None}
publishing_enabled = {"active": False}
autofit_enabled = {"active": True}
last_publish_time = {"t": 0.0}
publish_interval = {"hz": 100}
publish_timestamps = []
key_hold_start = {}
highlight_theme = None


class JointInterfaceNode(Node):
    def __init__(self):
        super().__init__("joint_interface_gui")
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            JointState, PUBLISH_TOPIC, self.qos_profile
        )
        self.subscription = self.create_subscription(
            JointState, SUBSCRIBE_TOPIC, self.joint_state_callback, self.qos_profile
        )

        self.joint_values = {name: 0.0 for name in JOINT_NAMES}
        self.interpolated_joint_values = {name: 0.0 for name in JOINT_NAMES}
        self.joint_efforts = {name: 0.0 for name in JOINT_NAMES}
        self.actual_joint_values = {name: 0.0 for name in JOINT_NAMES}
        self.ini = True

        self.sin_mag = 0.0
        self.sin_omega = 0.0
        self.sin_time = 0.0
        self.sin_ref_val = 0.0
        self.sin_mode = False
        self.sin_joint_name = None

    def publish_joint_command(self):
        self.interpolate()
        if self.sin_mode:
            for n in JOINT_NAMES:
                if self.sin_joint_name == n:
                    self.joint_values[n] = self.sin_ref_val + self.sin_mag * np.sin(
                        self.sin_omega * (time.time() - self.sin_time)
                    )
                    self.interpolated_joint_values[n] = copy.deepcopy(
                        self.joint_values[n]
                    )

                    dpg.set_value(f"{n}_slider", self.joint_values[n])
                    dpg.set_value(f"{n}_input", self.joint_values[n])

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [self.interpolated_joint_values[n] for n in JOINT_NAMES]
        msg.effort = [self.joint_efforts[n] for n in JOINT_NAMES]

        self.publisher.publish(msg)

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.actual_joint_values and i < len(msg.position):
                self.actual_joint_values[name] = msg.position[i]

        if self.ini:
            self.joint_values = copy.deepcopy(self.actual_joint_values)
            self.interpolated_joint_values = copy.deepcopy(self.actual_joint_values)
            self.ini = False

    def interpolate(self):
        for n in JOINT_NAMES:
            if self.joint_values[n] > self.interpolated_joint_values[n]:
                self.interpolated_joint_values[n] += JOINT_VEL / publish_interval["hz"]
                if self.interpolated_joint_values[n] > self.joint_values[n]:
                    self.interpolated_joint_values[n] = self.joint_values[n]
            elif self.joint_values[n] < self.interpolated_joint_values[n]:
                self.interpolated_joint_values[n] -= JOINT_VEL / publish_interval["hz"]
                if self.interpolated_joint_values[n] < self.joint_values[n]:
                    self.interpolated_joint_values[n] = self.joint_values[n]


def select_joint_callback(sender):
    if selected_joint["name"] == sender:
        # Deselect if already selected
        dpg.bind_item_theme(sender, 0)
        selected_joint["name"] = None
        ros_node.sin_mode = False
    else:
        for j in JOINT_NAMES:
            dpg.bind_item_theme(j, 0)
            dpg.bind_item_theme(f"Sin({j})", 0)
        dpg.bind_item_theme(sender, highlight_theme)
        selected_joint["name"] = sender
        ros_node.sin_mode = False


def select_sin_mode_callback(sender):
    if selected_joint["name"] == sender:
        # Deselect if already selected
        dpg.bind_item_theme(sender, 0)
        selected_joint["name"] = None
        ros_node.sin_mode = False
    else:
        for j in JOINT_NAMES:
            dpg.bind_item_theme(j, 0)
            dpg.bind_item_theme(f"Sin({j})", 0)
        dpg.bind_item_theme(sender, highlight_theme)
        selected_joint["name"] = sender
        ros_node.sin_time = time.time()

        for name in JOINT_NAMES:
            if name in sender:
                # ros_node.sin_ref_val = ros_node.actual_joint_values[name]
                ros_node.sin_ref_val = ros_node.interpolated_joint_values[name]
                ros_node.sin_joint_name = name
        ros_node.sin_mode = True


def keyboard_callback(sender, app_data):
    if not isinstance(app_data, (list, tuple)) or len(app_data) != 2:
        return

    key_code, is_down = app_data

    if is_down:
        now = ros_node.get_clock().now().nanoseconds / 1e9
        last_time = key_hold_start.get(key_code, 0)
        elapsed = now - last_time

        # Only allow once immediately or after hold threshold
        if elapsed < 0.1 and last_time != 0:
            return
        key_hold_start[key_code] = now

        if key_code == 524:  ## space
            start_publishing()
        elif key_code == 526:  ## exc
            stop_publishing()

        joint = selected_joint["name"]
        if joint is None:
            return

        if key_code == dpg.mvKey_Up:
            delta = KEYBOARD_DELTA_VAL
        elif key_code == dpg.mvKey_Down:
            delta = -KEYBOARD_DELTA_VAL
        else:
            return

        val = ros_node.joint_values[joint] + delta
        min_val, max_val = JOINT_LIMITS[joint]
        val = max(min_val, min(max_val, val))
        ros_node.joint_values[joint] = val

        dpg.set_value(f"{joint}_slider", val)
        dpg.set_value(f"{joint}_input", val)


def clear_last_key():
    key_hold_start.clear()


def slider_callback(sender):
    joint = sender.replace("_slider", "")
    val = dpg.get_value(sender)
    ros_node.joint_values[joint] = val
    dpg.set_value(f"{joint}_slider", val)
    dpg.set_value(f"{joint}_input", val)


def input_callback(sender):
    joint = sender.replace("_input", "")
    val = dpg.get_value(sender)
    min_val, max_val = JOINT_LIMITS[joint]
    val = max(min_val, min(max_val, val))
    ros_node.joint_values[joint] = val
    print(val)
    dpg.set_value(f"{joint}_slider", val)
    dpg.set_value(f"{joint}_input", val)


def sin_mag_callback(sender):
    val = dpg.get_value(sender)
    if not ros_node.sin_mode:
        dpg.set_value(f"sin_mag", val)
        ros_node.sin_mag = val
    else:
        dpg.set_value(f"sin_mag", ros_node.sin_mag)


def sin_omega_callback(sender):
    val = dpg.get_value(sender)
    if not ros_node.sin_mode:
        dpg.set_value(f"sin_omega", val)
        ros_node.sin_omega = val
    else:
        dpg.set_value(f"sin_omega", ros_node.sin_omega)


def update_joint_effort(sender):
    joint = sender.replace("_effort", "")
    ros_node.joint_efforts[joint] = dpg.get_value(sender)


def set_all_efforts(sender):
    val = dpg.get_value("Effort All")
    for joint in JOINT_NAMES:
        ros_node.joint_efforts[joint] = val
        dpg.set_value(f"{joint}_effort", val)


def create_highlight_theme():
    with dpg.theme() as theme_id:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (0, 150, 250, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (0, 200, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (0, 100, 200, 255))
    return theme_id


def update_publish_interval(sender):
    val = dpg.get_value(sender)
    publish_interval["hz"] = min(100, val)


def start_publishing():
    publishing_enabled["active"] = True
    print("Publishing started.")
    dpg.bind_item_theme("pub_start_button", highlight_theme)
    dpg.bind_item_theme("pub_stop_button", 0)


def stop_publishing():
    publishing_enabled["active"] = False
    print("Publishing stopped.")
    dpg.bind_item_theme("pub_stop_button", highlight_theme)
    dpg.bind_item_theme("pub_start_button", 0)


def auto_fit():
    if autofit_enabled["active"]:
        autofit_enabled["active"] = False
        dpg.bind_item_theme("Autofit", 0)
    else:
        autofit_enabled["active"] = True
        dpg.bind_item_theme("Autofit", highlight_theme)


def update_all():
    now = ros_node.get_clock().now().nanoseconds / 1e9
    for joint in JOINT_NAMES:
        joint_histories[joint].append(ros_node.interpolated_joint_values[joint])
        if len(joint_histories[joint]) > LEN_JOINT_HISTORIES:
            joint_histories[joint].pop(0)
        dpg.set_value(
            f"{joint}_line_cmd",
            [list(range(len(joint_histories[joint]))), joint_histories[joint]],
        )

        actual_histories[joint].append(ros_node.actual_joint_values[joint])
        if len(actual_histories[joint]) > LEN_JOINT_HISTORIES:
            actual_histories[joint].pop(0)
        dpg.set_value(
            f"{joint}_line_actual",
            [list(range(len(actual_histories[joint]))), actual_histories[joint]],
        )

        if dpg.does_item_exist(f"{joint}_actual_text"):
            dpg.set_value(
                f"{joint}_actual_text",
                f"Actual {joint}: {ros_node.actual_joint_values[joint]:.3f}",
            )

        if dpg.does_item_exist(f"{joint}_interpolated_input"):
            dpg.set_value(
                f"{joint}_interpolated_input",
                f"{ros_node.interpolated_joint_values[joint]:.3f}",
            )

        if autofit_enabled["active"]:
            dpg.fit_axis_data(f"{joint}_yaxis")
            min_val = (
                min(
                    min(actual_histories[joint]),
                    min(joint_histories[joint]),
                )
                - 0.05
            )
            max_val = (
                max(
                    max(actual_histories[joint]),
                    max(joint_histories[joint]),
                )
                + 0.05
            )
            dpg.set_axis_limits(f"{joint}_yaxis", min_val, max_val)

            dpg.fit_axis_data(f"{joint}_xaxis")
            dpg.set_axis_limits(f"{joint}_xaxis", 0, LEN_JOINT_HISTORIES)
        else:
            dpg.set_axis_limits_auto(f"{joint}_xaxis")
            dpg.set_axis_limits_auto(f"{joint}_yaxis")

    # rclpy.spin_once(ros_node, timeout_sec=0)
    if publishing_enabled["active"] and (
        now - last_publish_time["t"] >= 1 / publish_interval["hz"]
    ):
        ros_node.publish_joint_command()
        last_publish_time["t"] = now
        publish_timestamps.append(now)
        if len(publish_timestamps) > 10:
            publish_timestamps.pop(0)

    if len(publish_timestamps) >= 2 and dpg.does_item_exist("Actual Publish Hz"):
        intervals = [
            t2 - t1 for t1, t2 in zip(publish_timestamps[:-1], publish_timestamps[1:])
        ]
        avg_interval = sum(intervals) / len(intervals)
        actual_hz = 1.0 / avg_interval if avg_interval > 0 else 0.0
        dpg.set_value("Actual Publish Hz", f"Actual: {actual_hz:.2f} Hz")

    if not publishing_enabled["active"]:
        actual_hz = 0.0
        dpg.set_value("Actual Publish Hz", f"Actual: {actual_hz:.2f} Hz")


def setup_ui():
    def ros_spin_loop():
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.01)

    threading.Thread(target=ros_spin_loop, daemon=True).start()

    while ros_node.ini:
        print("Cannot subscribe robot joint state")
        time.sleep(0.5)

    with dpg.window(label="Joint Control Panel", tag="main_window"):
        with dpg.group(horizontal=True):
            with dpg.child_window(width=600):

                with dpg.group(horizontal=True):
                    with dpg.group():
                        dpg.add_text("Select Joint", tag="text_joint_select")
                        for name in JOINT_NAMES:
                            dpg.add_button(
                                label=name, tag=name, callback=select_joint_callback
                            )

                    dpg.add_spacer(width=20)
                    with dpg.group():
                        dpg.add_text("Sinusoidal Motion", tag="text_sin_motion")
                        for name in JOINT_NAMES:
                            dpg.add_button(
                                label=f"Sin({name})",
                                tag=f"Sin({name})",
                                callback=select_sin_mode_callback,
                            )

                    dpg.add_spacer(width=10)
                    with dpg.group():
                        dpg.add_spacer(height=70)
                        with dpg.group(horizontal=True):
                            dpg.add_text(
                                f"Sin Magnitude:",
                            )
                            dpg.add_input_float(
                                tag="sin_mag",
                                default_value=0.0,
                                callback=sin_mag_callback,
                                width=100,
                                on_enter=True,
                            )
                        with dpg.group(horizontal=True):
                            dpg.add_text(
                                f"Sin Omega:",
                            )
                            dpg.add_input_float(
                                tag="sin_omega",
                                default_value=0.0,
                                callback=sin_omega_callback,
                                width=100,
                                on_enter=True,
                            )

                dpg.add_spacer(height=10)
                dpg.add_text("Joint Commands", tag="text_joint_commands")
                for name in JOINT_NAMES:
                    min_val, max_val = JOINT_LIMITS[name]
                    with dpg.group(horizontal=True):
                        dpg.add_text(f"{name}", bullet=True)
                        dpg.add_slider_float(
                            tag=f"{name}_slider",
                            # label=f"{name} ({min_val:.2f} ~ {max_val:.2f})",
                            # label=f"{name}",
                            default_value=ros_node.actual_joint_values[name],
                            min_value=min_val,
                            max_value=max_val,
                            callback=slider_callback,
                            user_data=name,
                            width=300,
                            format="%.3f",
                            clamped=True,
                        )
                        dpg.add_input_float(
                            tag=f"{name}_input",
                            default_value=ros_node.actual_joint_values[name],
                            callback=input_callback,
                            width=100,
                            on_enter=True,
                        )
                        dpg.add_text(
                            tag=f"{name}_interpolated_input",
                            default_value=ros_node.actual_joint_values[name],
                        )

                dpg.add_spacer(height=10)
                dpg.add_text("Joint States", tag="text_joint_states")
                for joint in JOINT_NAMES:
                    dpg.add_text(
                        f"Actual: 0.000", tag=f"{joint}_actual_text", bullet=True
                    )

                dpg.add_spacer(height=10)
                dpg.add_text("Individual Effort Settings", tag="text_effort")
                for name in JOINT_NAMES:
                    dpg.add_input_float(
                        tag=f"{name}_effort",
                        label=f"{name} Effort",
                        default_value=0.0,
                        callback=update_joint_effort,
                        width=350,
                        on_enter=True,
                    )
                dpg.add_input_float(
                    tag="Effort All",
                    label="Effort All",
                    default_value=0.0,
                    callback=set_all_efforts,
                    width=350,
                    on_enter=True,
                )

                dpg.add_spacer(height=10)
                dpg.add_text("Publishing Control", tag="text_control")
                with dpg.group(horizontal=True):
                    dpg.add_button(
                        label="Start (SPACE)",
                        callback=start_publishing,
                        tag="pub_start_button",
                    )
                    dpg.add_button(
                        label="Stop (ESC)",
                        callback=stop_publishing,
                        tag="pub_stop_button",
                    )
                dpg.bind_item_theme("pub_stop_button", highlight_theme)

                with dpg.group(horizontal=False):
                    dpg.add_input_float(
                        tag="Publish Interval (Hz)",
                        default_value=publish_interval["hz"],
                        callback=update_publish_interval,
                        on_enter=True,
                    )
                    dpg.add_text("", tag="Actual Publish Hz")

            with dpg.child_window(width=600, tag="second_child_window"):
                dpg.add_text("Realtime Plot", tag="text_graph")
                dpg.add_button(label="AutoFit", callback=auto_fit, tag="Autofit")
                dpg.bind_item_theme("Autofit", highlight_theme)
                for name in JOINT_NAMES:
                    min_val, max_val = JOINT_LIMITS[name]
                    with dpg.plot(
                        label=f"{name}_plot", height=160, width=-1, tag=f"{name}_plot"
                    ):
                        dpg.add_plot_legend()
                        # dpg.add_plot_axis(dpg.mvXAxis, label="", auto_fit=True)
                        # with dpg.plot_axis(
                        #     dpg.mvYAxis, label="", auto_fit=True
                        # ) as y_axis:
                        dpg.add_plot_axis(dpg.mvXAxis, label="", tag=f"{name}_xaxis")
                        with dpg.plot_axis(
                            dpg.mvYAxis, label="", tag=f"{name}_yaxis"
                        ) as y_axis:
                            # dpg.set_axis_limits(y_axis, min_val, max_val)
                            dpg.add_line_series(
                                list(range(100)),
                                joint_histories[name],
                                label="command",
                                tag=f"{name}_line_cmd",
                            )
                            dpg.add_line_series(
                                list(range(100)),
                                actual_histories[name],
                                label="actual",
                                tag=f"{name}_line_actual",
                            )
                            line_thickness_theme = create_line_thickness_theme(3)
                            dpg.bind_item_theme(
                                f"{name}_line_cmd", line_thickness_theme
                            )
                            dpg.bind_item_theme(
                                f"{name}_line_actual", line_thickness_theme
                            )

    dpg.create_viewport(
        title="Joint Controller", width=1220, height=1200, resizable=True
    )
    dpg.setup_dearpygui()
    dpg.set_primary_window("main_window", True)
    with dpg.handler_registry():
        dpg.add_key_down_handler(callback=keyboard_callback)
        dpg.add_key_release_handler(callback=lambda s, a: clear_last_key())
    dpg.show_viewport()

    def run_update_loop():
        while dpg.is_dearpygui_running():
            update_all()
            time.sleep(1.0 / 100)

    threading.Thread(target=run_update_loop, daemon=True).start()
    dpg.start_dearpygui()


def create_line_thickness_theme(thickness=5.0):
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvLineSeries):
            dpg.add_theme_style(
                dpg.mvPlotStyleVar_LineWeight, thickness, category=dpg.mvThemeCat_Plots
            )

    return theme


def main():
    global ros_node, highlight_theme
    rclpy.init()
    ros_node = JointInterfaceNode()
    dpg.create_context()
    highlight_theme = create_highlight_theme()
    setup_ui()


if __name__ == "__main__":
    main()
