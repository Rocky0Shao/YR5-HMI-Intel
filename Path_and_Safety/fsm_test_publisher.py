#!/usr/bin/env python3
"""
fsm_test_publisher.py - Interactive FSM State Test Publisher

Publishes dummy FSM states to test safety_comms_node.

ROS Publications:
    - /autodrive/fsm/state (std_msgs/Int32) - FSM state number
    - /autodrive/fsm/description (std_msgs/String) - FSM state description

Usage:
    python3 fsm_test_publisher.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

import threading

# FSM state definitions (from safety.py)
FSM_STATES = {
    0: "STATE0",
    1: "STARTUP",
    2: "PASSIVE_MODE",
    3: "ACTIVATION_CONDITION",
    4: "BRAKE_ACTIVATION",
    5: "WAIT_BRAKE_RELEASE",
    6: "STEER_ACTIVATION",
    7: "PROPULSION_ACTIVATION",
    8: "AV",
    9: "DEACTIVATION",
    10: "ACTIVATION_FAILURE",
}


class FSMTestPublisher(Node):
    def __init__(self):
        super().__init__('fsm_test_publisher')

        self.state_pub = self.create_publisher(Int32, '/autodrive/fsm/state', 10)
        self.desc_pub = self.create_publisher(String, '/autodrive/fsm/description', 10)

        self.get_logger().info('FSM Test Publisher started')
        self._print_menu()

        # Run interactive input on a separate thread so ROS can spin
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()

    def _print_menu(self):
        print("\n=== FSM Test Publisher ===")
        print("Enter a state number to publish:\n")
        for num, name in FSM_STATES.items():
            print(f"  {num:2d} = {name}")
        print("\n  q  = quit\n")

    def _publish_state(self, state_num: int):
        description = FSM_STATES.get(state_num, f"UNKNOWN_{state_num}")

        # Publish state as Int32
        state_msg = Int32()
        state_msg.data = state_num
        self.state_pub.publish(state_msg)

        # Publish description
        desc_msg = String()
        desc_msg.data = description
        self.desc_pub.publish(desc_msg)

        self.get_logger().info(f'Published: state={state_num}, description="{description}"')

    def _input_loop(self):
        try:
            while rclpy.ok():
                raw = input("State> ").strip()
                if raw.lower() == 'q':
                    print("[quitting]")
                    rclpy.shutdown()
                    break

                try:
                    state_num = int(raw)
                except ValueError:
                    print(f"Invalid input: '{raw}'. Enter a number 0-10 or 'q'.")
                    continue

                if state_num not in FSM_STATES:
                    print(f"Warning: {state_num} is not a known FSM state (0-10), publishing anyway.")

                self._publish_state(state_num)

        except (EOFError, KeyboardInterrupt):
            print("\n[interrupted]")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FSMTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
