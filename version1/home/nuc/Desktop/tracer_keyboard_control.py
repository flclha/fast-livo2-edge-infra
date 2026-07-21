#!/usr/bin/env python3
import os
import select
import sys
import termios
import tty

try:
    import tkinter as tk
except Exception:
    tk = None

import rospy
from geometry_msgs.msg import Twist


DIRECTION_KEYS = ("w", "s", "a", "d")
KEY_LABELS = {
    "w": "forward",
    "s": "backward",
    "a": "turn left",
    "d": "turn right",
}

TERMINAL_HELP = """Tracer keyboard control

w: forward 0.1 m/s
s: backward 0.1 m/s
a: turn left in place at 0.3 rad/s
d: turn right in place at 0.3 rad/s
release key: stop in GUI mode; terminal mode uses a deadman timeout
space/x: stop immediately
q/Ctrl-C: stop and quit
"""


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


def make_twist(linear=0.0, angular=0.0):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg


def command_for_key(key, speed, turn):
    if key == "w":
        return make_twist(speed, 0.0)
    if key == "s":
        return make_twist(-speed, 0.0)
    if key == "a":
        return make_twist(0.0, turn)
    if key == "d":
        return make_twist(0.0, -turn)
    return make_twist()


def publish_stop(pub):
    stop = make_twist()
    for _ in range(5):
        pub.publish(stop)
        rospy.sleep(0.05)


def has_display():
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def run_gui(pub, topic, speed, turn, publish_hz):
    if tk is None:
        raise RuntimeError("tkinter is not available")

    root = tk.Tk()
    root.title("Tracer keyboard control")
    root.geometry("420x220")

    state = {
        "current": make_twist(),
        "active_key": None,
        "closed": False,
    }
    pressed = set()
    press_order = []
    key_epoch = {}

    info = (
        "Focus this window and hold keys:\n"
        "w/s: +/-0.1 m/s\n"
        "a/d: +/-0.3 rad/s in place\n"
        "Release key: stop\n"
        "space/x: stop, q/Esc: quit"
    )
    tk.Label(root, text=info, justify="left", padx=16, pady=12).pack(anchor="w")
    status_var = tk.StringVar(value="stopped")
    tk.Label(root, textvariable=status_var, font=("Sans", 16, "bold"), padx=16).pack(anchor="w")
    topic_var = tk.StringVar(value="Publishing to {}".format(topic))
    tk.Label(root, textvariable=topic_var, padx=16, pady=8).pack(anchor="w")

    def set_stop():
        state["current"] = make_twist()
        state["active_key"] = None
        status_var.set("stopped")

    def remove_from_order(key):
        while key in press_order:
            press_order.remove(key)

    def activate_latest_pressed():
        while press_order and press_order[-1] not in pressed:
            press_order.pop()
        if press_order:
            key = press_order[-1]
            state["active_key"] = key
            state["current"] = command_for_key(key, speed, turn)
            status_var.set(KEY_LABELS[key])
        else:
            set_stop()

    def on_press(event):
        key = event.keysym.lower()
        if key in DIRECTION_KEYS:
            key_epoch[key] = key_epoch.get(key, 0) + 1
            pressed.add(key)
            remove_from_order(key)
            press_order.append(key)
            activate_latest_pressed()
        elif key in ("space", "x"):
            pressed.clear()
            press_order.clear()
            set_stop()
        elif key in ("q", "escape"):
            close_window()

    def on_release(event):
        key = event.keysym.lower()
        if key not in DIRECTION_KEYS:
            return

        epoch = key_epoch.get(key, 0)

        def finish_release():
            if key_epoch.get(key, 0) != epoch:
                return
            pressed.discard(key)
            remove_from_order(key)
            activate_latest_pressed()

        root.after(80, finish_release)

    def publish_loop():
        if state["closed"] or rospy.is_shutdown():
            return
        pub.publish(state["current"])
        root.after(max(1, int(1000.0 / publish_hz)), publish_loop)

    def close_window():
        if state["closed"]:
            return
        state["closed"] = True
        set_stop()
        publish_stop(pub)
        root.destroy()

    root.bind("<KeyPress>", on_press)
    root.bind("<KeyRelease>", on_release)
    root.protocol("WM_DELETE_WINDOW", close_window)
    root.after(100, publish_loop)
    root.after(200, root.focus_force)
    root.mainloop()
    publish_stop(pub)


def run_terminal(pub, topic, speed, turn, publish_hz, hold_timeout):
    rate = rospy.Rate(publish_hz)
    current = make_twist()
    last_motion_key_time = rospy.Time(0)
    stopped = True

    old_settings = termios.tcgetattr(sys.stdin)
    print(TERMINAL_HELP)
    print(
        "Publishing to {}. speed={:.2f} m/s, turn={:.2f} rad/s, hold_timeout={:.2f}s".format(
            topic, speed, turn, hold_timeout
        )
    )

    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            key = read_key(1.0 / publish_hz)
            now = rospy.Time.now()
            if key in ("w", "W"):
                current = command_for_key("w", speed, turn)
                last_motion_key_time = now
                stopped = False
            elif key in ("s", "S"):
                current = command_for_key("s", speed, turn)
                last_motion_key_time = now
                stopped = False
            elif key in ("a", "A"):
                current = command_for_key("a", speed, turn)
                last_motion_key_time = now
                stopped = False
            elif key in ("d", "D"):
                current = command_for_key("d", speed, turn)
                last_motion_key_time = now
                stopped = False
            elif key in (" ", "x", "X"):
                current = make_twist()
                stopped = True
            elif key in ("q", "Q", "\x03"):
                break

            if not stopped and (now - last_motion_key_time).to_sec() > hold_timeout:
                current = make_twist()
                stopped = True

            pub.publish(current)
            rate.sleep()
    finally:
        publish_stop(pub)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("stopped")


def main():
    rospy.init_node("tracer_keyboard_control")
    topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
    speed = rospy.get_param("~speed", 0.1)
    turn = rospy.get_param("~turn", 0.3)
    publish_hz = rospy.get_param("~publish_hz", 20.0)
    hold_timeout = rospy.get_param("~hold_timeout", 0.25)
    mode = rospy.get_param("~mode", os.environ.get("TRACER_KEYBOARD_MODE", "auto")).lower()

    pub = rospy.Publisher(topic, Twist, queue_size=1)

    if mode not in ("auto", "gui", "terminal"):
        raise ValueError("mode must be auto, gui, or terminal")

    use_gui = mode == "gui" or (mode == "auto" and has_display() and tk is not None)
    if use_gui:
        try:
            run_gui(pub, topic, speed, turn, publish_hz)
            return
        except Exception as exc:
            if mode == "gui":
                raise
            rospy.logwarn("GUI keyboard control unavailable, falling back to terminal: %s", exc)

    run_terminal(pub, topic, speed, turn, publish_hz, hold_timeout)


if __name__ == "__main__":
    main()
