"""
safety_comms_node.py - Safety Communications Node

Bidirectional communication node for safety FSM state and HMI commands.

RX (HMI -> Intel) on port 6001:
    - Receives HMITxMessage (engage_status, target_destination)
    - Intel listens as server, HMI connects as client

TX (Intel -> HMI) on port 5003:
    - Sends SafetyStatus (state, description)
    - Intel connects as client, HMI listens as server
    - Sends on every state change

ROS Subscriptions:
    - /autodrive/fsm/state (std_msgs/Int32) - FSM state number
    - /autodrive/fsm/description (std_msgs/String) - FSM state description

ROS Publications:
    - /safety/engage_state (std_msgs/Int32) - 0=disengage, 1=engage, 3=disabled
    - /controls/target_destination (std_msgs/String) - destination letter A-Z
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from typing import Optional
import socket
import struct
import threading
import select

import HMI_TX_CONTROLS_pb2 as hmi_tx
import SAFETY_STATUS_pb2 as safety_pb


class SafetyCommsNode(Node):
    def __init__(self):
        super().__init__('safety_comms_node')

        # --- ROS Subscribers ---
        self.create_subscription(
            Int32,
            '/autodrive/fsm/state',
            self.cb_fsm_state,
            10
        )
        self.create_subscription(
            String,
            '/autodrive/fsm/description',
            self.cb_fsm_description,
            10
        )

        # --- ROS Publishers ---
        self.engage_pub = self.create_publisher(
            Int32,
            '/safety/engage_state',
            10
        )
        self.dest_pub = self.create_publisher(
            String,
            '/controls/target_destination',
            10
        )

        # --- State tracking ---
        self.current_fsm_state: int = 0
        self.current_fsm_description: str = ""
        self.last_sent_state: Optional[int] = None
        self.last_sent_description: Optional[str] = None

        # --- TCP TX: Intel -> HMI (SafetyStatus on port 5003) ---
        self.declare_parameter('hmi_tx_host', '127.0.0.1')
        self.declare_parameter('hmi_tx_port', 5003)
        self.hmi_tx_host: str = self.get_parameter('hmi_tx_host').value
        self.hmi_tx_port: int = int(self.get_parameter('hmi_tx_port').value)
        self.hmi_tx_sock: Optional[socket.socket] = None

        # --- TCP RX: HMI -> Intel (HMITxMessage on port 6001) ---
        self.declare_parameter('hmi_rx_port', 6001)
        self.hmi_rx_port: int = int(self.get_parameter('hmi_rx_port').value)
        self.hmi_rx_server: Optional[socket.socket] = None
        self.hmi_rx_client: Optional[socket.socket] = None
        self.hmi_rx_buf: bytes = b''

        # Start RX server
        self._start_rx_server()

        # Timer to check for incoming commands (20 Hz)
        self.rx_timer = self.create_timer(0.05, self.rx_step)

        self.get_logger().info(
            f'SafetyCommsNode started - TX to {self.hmi_tx_host}:{self.hmi_tx_port}, '
            f'RX server on port {self.hmi_rx_port}'
        )

    # --------------------------------------------------------------------------
    # TCP TX: Intel -> HMI (SafetyStatus)
    # --------------------------------------------------------------------------

    def _connect_hmi_tx(self) -> None:
        """Connect to HMI for SafetyStatus transmission."""
        if self.hmi_tx_sock:
            try:
                self.hmi_tx_sock.close()
            except Exception:
                pass
            self.hmi_tx_sock = None

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(2.0)
            s.connect((self.hmi_tx_host, self.hmi_tx_port))
            s.settimeout(0.5)
            self.hmi_tx_sock = s
            self.get_logger().info(f'Connected SafetyStatus TX to {self.hmi_tx_host}:{self.hmi_tx_port}')
        except Exception as e:
            self.hmi_tx_sock = None
            self.get_logger().warning(f'SafetyStatus TX connect failed: {e}')

    def _send_safety_status(self) -> None:
        """Send SafetyStatus protobuf to HMI (length-prefixed)."""
        # Only send if state or description changed
        if (self.current_fsm_state == self.last_sent_state and
                self.current_fsm_description == self.last_sent_description):
            return

        try:
            # Build protobuf
            status = safety_pb.SafetyStatus()
            status.state = self.current_fsm_state
            status.description = self.current_fsm_description
            payload = status.SerializeToString()

            if not payload:
                return

            # Connect if needed
            if self.hmi_tx_sock is None:
                self._connect_hmi_tx()
            if self.hmi_tx_sock is None:
                return

            # Send length-prefixed frame
            frame = struct.pack(">I", len(payload)) + payload
            self.hmi_tx_sock.sendall(frame)

            # Update last sent values
            self.last_sent_state = self.current_fsm_state
            self.last_sent_description = self.current_fsm_description

            self.get_logger().info(
                f'Sent SafetyStatus: state={self.current_fsm_state}, '
                f'description="{self.current_fsm_description}"'
            )

        except (socket.timeout, ConnectionRefusedError,
                ConnectionResetError, BrokenPipeError) as e:
            self.get_logger().warning(f'SafetyStatus TX failed, will retry: {e}')
            self._connect_hmi_tx()
        except Exception as e:
            self.get_logger().error(f'SafetyStatus TX error: {e}')

    # --------------------------------------------------------------------------
    # TCP RX: HMI -> Intel (HMITxMessage) - Server mode
    # --------------------------------------------------------------------------

    def _start_rx_server(self) -> None:
        """Start TCP server to receive HMI commands."""
        try:
            self.hmi_rx_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.hmi_rx_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.hmi_rx_server.bind(('0.0.0.0', self.hmi_rx_port))
            self.hmi_rx_server.listen(1)
            self.hmi_rx_server.setblocking(False)
            self.get_logger().info(f'HMI RX server listening on port {self.hmi_rx_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to start RX server: {e}')
            self.hmi_rx_server = None

    def _accept_rx_client(self) -> None:
        """Accept incoming client connection."""
        if self.hmi_rx_server is None:
            return

        try:
            readable, _, _ = select.select([self.hmi_rx_server], [], [], 0)
            if readable:
                client, addr = self.hmi_rx_server.accept()
                client.setblocking(False)
                # Close any existing client
                if self.hmi_rx_client:
                    try:
                        self.hmi_rx_client.close()
                    except Exception:
                        pass
                self.hmi_rx_client = client
                self.hmi_rx_buf = b''
                self.get_logger().info(f'HMI client connected from {addr}')
        except Exception:
            pass

    def _handle_hmi_command(self, cmd) -> None:
        """Process received HMITxMessage and publish to ROS topics."""
        # Map engage_status (0=DISENGAGE, 1=ENGAGE, 2=DISABLED)
        status = int(cmd.engage_status)
        if status not in (0, 1, 2):
            self.get_logger().warning(f'Invalid engage_status: {status}')
            return

        # Map DISABLED=2 -> 3 for /safety/engage_state
        if status == 2:
            status = 3

        # Publish engage state
        engage_msg = Int32()
        engage_msg.data = status
        self.engage_pub.publish(engage_msg)

        # Publish destination if provided
        if cmd.target_destination:
            dest_msg = String()
            dest_msg.data = cmd.target_destination
            self.dest_pub.publish(dest_msg)

        self.get_logger().info(
            f"HMI command -> engage_state={status}, "
            f"destination='{cmd.target_destination or '(empty)'}'"
        )

    def rx_step(self) -> None:
        """Poll for incoming HMI commands."""
        # Try to accept new client
        self._accept_rx_client()

        if self.hmi_rx_client is None:
            return

        try:
            # Non-blocking read
            readable, _, _ = select.select([self.hmi_rx_client], [], [], 0)
            if not readable:
                return

            chunk = self.hmi_rx_client.recv(4096)
            if not chunk:
                # Client disconnected
                self.get_logger().warning('HMI client disconnected')
                try:
                    self.hmi_rx_client.close()
                except Exception:
                    pass
                self.hmi_rx_client = None
                self.hmi_rx_buf = b''
                return

            self.hmi_rx_buf += chunk

            # Parse length-prefixed frames
            MAX_MSG_LEN = 1024 * 1024
            while True:
                if len(self.hmi_rx_buf) < 4:
                    break
                msg_len = struct.unpack(">I", self.hmi_rx_buf[:4])[0]
                if msg_len > MAX_MSG_LEN:
                    self.get_logger().error(f'Message too large: {msg_len}')
                    self.hmi_rx_buf = b''
                    break
                if len(self.hmi_rx_buf) < 4 + msg_len:
                    break

                frame = self.hmi_rx_buf[4:4 + msg_len]
                self.hmi_rx_buf = self.hmi_rx_buf[4 + msg_len:]

                try:
                    cmd = hmi_tx.HMITxMessage()
                    cmd.ParseFromString(frame)
                    self._handle_hmi_command(cmd)
                except Exception as e:
                    self.get_logger().error(f'Failed to parse HMITxMessage: {e}')

        except Exception as e:
            self.get_logger().warning(f'RX error: {e}')
            try:
                if self.hmi_rx_client:
                    self.hmi_rx_client.close()
            except Exception:
                pass
            self.hmi_rx_client = None
            self.hmi_rx_buf = b''

    # --------------------------------------------------------------------------
    # ROS Callbacks
    # --------------------------------------------------------------------------

    def cb_fsm_state(self, msg: Int32) -> None:
        """Handle FSM state update from safety node."""
        self.current_fsm_state = msg.data
        self._send_safety_status()

    def cb_fsm_description(self, msg: String) -> None:
        """Handle FSM description update from safety node."""
        self.current_fsm_description = msg.data
        self._send_safety_status()

    # --------------------------------------------------------------------------
    # Cleanup
    # --------------------------------------------------------------------------

    def destroy_node(self):
        """Clean up sockets on shutdown."""
        if self.hmi_tx_sock:
            try:
                self.hmi_tx_sock.close()
            except Exception:
                pass

        if self.hmi_rx_client:
            try:
                self.hmi_rx_client.close()
            except Exception:
                pass

        if self.hmi_rx_server:
            try:
                self.hmi_rx_server.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCommsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
