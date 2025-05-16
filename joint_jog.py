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

JOINT_NAMES = [f"joint{i+1}" for i in range(7)]
JOINT_LIMITS = {name: (-3.14, 3.14) for name in JOINT_NAMES}
PUBLISH_TOPIC = "/arm/target_joint_state"
SUBSCRIBE_TOPIC = "/arm/joint_states"
JOINT_VEL = 0.1  ## rad/s
KEYBOARD_DELTA_VAL = 0.05  ## rad

joint_histories = {name: [0.0] * 100 for name in JOINT_NAMES}
actual_histories = {name: [0.0] * 100 for name in JOINT_NAMES}
selected_joint = {"name": None}
publishing_enabled = {"active": False}
last_publish_time = {"t": 0.0}
publish_interval = {"hz": 10}
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

    def publish_joint_command(self):
        self.interpolate()
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
    else:
        for j in JOINT_NAMES:
            dpg.bind_item_theme(j, 0)
        dpg.bind_item_theme(sender, highlight_theme)
        selected_joint["name"] = sender


def keyboard_callback(sender, app_data):
    if not isinstance(app_data, (list, tuple)) or len(app_data) != 2:
        return

    key_code, is_down = app_data
    joint = selected_joint["name"]
    if joint is None:
        return

    if is_down:
        now = ros_node.get_clock().now().nanoseconds / 1e9
        last_time = key_hold_start.get(key_code, 0)
        elapsed = now - last_time

        # Only allow once immediately or after hold threshold
        if elapsed < 0.05 and last_time != 0:
            return
        key_hold_start[key_code] = now

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
    dpg.set_value(f"{joint}_slider", val)
    dpg.set_value(f"{joint}_input", val)


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


def stop_publishing():
    publishing_enabled["active"] = False
    print("Publishing stopped.")


def update_all():
    now = ros_node.get_clock().now().nanoseconds / 1e9
    for joint in JOINT_NAMES:
        joint_histories[joint].append(ros_node.interpolated_joint_values[joint])
        if len(joint_histories[joint]) > 100:
            joint_histories[joint].pop(0)
        dpg.set_value(
            f"{joint}_line_cmd",
            [list(range(len(joint_histories[joint]))), joint_histories[joint]],
        )

        actual_histories[joint].append(ros_node.actual_joint_values[joint])
        if len(actual_histories[joint]) > 100:
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
                f"{joint}_interpolated_input", ros_node.interpolated_joint_values[joint]
            )

    rclpy.spin_once(ros_node, timeout_sec=0)
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
    import threading

    def ros_spin_loop():
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.01)

    def run_update_loop():
        while dpg.is_dearpygui_running():
            update_all()
            time.sleep(1.0 / 30)

    threading.Thread(target=ros_spin_loop, daemon=True).start()
    threading.Thread(target=run_update_loop, daemon=True).start()
    with dpg.window(label="Joint Control Panel", tag="main_window"):
        with dpg.group(horizontal=True):
            with dpg.child_window(width=600) as left_child:
                dpg.add_text("Select Joint", tag="text_joint_select")
                with dpg.group(horizontal=True):
                    for name in JOINT_NAMES:
                        dpg.add_button(
                            label=name, tag=name, callback=select_joint_callback
                        )
                dpg.add_spacer(height=10)
                dpg.add_text("Joint Commands", tag="text_joint_commands")
                for name in JOINT_NAMES:
                    min_val, max_val = JOINT_LIMITS[name]
                    with dpg.group(horizontal=True):
                        dpg.add_slider_float(
                            tag=f"{name}_slider",
                            # label=f"{name} ({min_val:.2f} ~ {max_val:.2f})",
                            label=f"{name}",
                            default_value=0.0,
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
                            default_value=0.0,
                            callback=input_callback,
                            width=100,
                        )
                        dpg.add_text(
                            tag=f"{name}_interpolated_input",
                            default_value=0.0,
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
                    )
                dpg.add_input_float(
                    tag="Effort All",
                    label="Effort All",
                    default_value=0.0,
                    callback=set_all_efforts,
                    width=350,
                )

                dpg.add_spacer(height=10)
                dpg.add_text("Publishing Control", tag="text_control")
                with dpg.group(horizontal=True):
                    dpg.add_button(label="Start", callback=start_publishing)
                    dpg.add_button(label="Stop", callback=stop_publishing)
                with dpg.group(horizontal=False):
                    dpg.add_input_float(
                        tag="Publish Interval (Hz)",
                        default_value=10,
                        callback=update_publish_interval,
                        on_enter=True,
                    )
                    dpg.add_text("", tag="Actual Publish Hz")

            with dpg.child_window(width=600):
                dpg.add_text("Realtime Plot", tag="text_graph")
                for name in JOINT_NAMES:
                    min_val, max_val = JOINT_LIMITS[name]
                    with dpg.plot(label=f"{name} Plot", height=160, width=-1):
                        dpg.add_plot_axis(dpg.mvXAxis, label="")
                        with dpg.plot_axis(dpg.mvYAxis, label="") as y_axis:
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

    import threading

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
