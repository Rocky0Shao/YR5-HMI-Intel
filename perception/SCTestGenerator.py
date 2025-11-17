import random
import time
from pathlib import Path

import can
import cantools

# Load the DBC file
dbc_file_path = "ADC2_SC_2024.2.dbc"
db = cantools.database.load_file(dbc_file_path)

# Output file for generated CAN messages
output_path = Path("generated_can_messages.txt")


def _append_to_output(line: str) -> None:
    """Append a single log line to the output text file."""

    with output_path.open("a", encoding="ascii") as out_file:
        out_file.write(line + "\n")

# Define a function to generate random CAN messages within the valid range
def random_can_single():
    while True:
        # Choose a random message from the DBC file
        message = random.choice(db.messages)

        # Create random data for the message fields within valid ranges
        data = {}
        for signal in message.signals:
            min_val = signal.minimum if signal.minimum is not None else 0
            max_val = signal.maximum if signal.maximum is not None else (2**signal.length - 1)
            data[signal.name] = random.randint(int(min_val), int(max_val))

        # Encode the message data
        try:
            encoded_data = message.encode(data)
            # Create a CAN message object for parity with the original behaviour.
            can_msg = can.Message(
                arbitration_id=message.frame_id,
                data=encoded_data,
                is_extended_id=False,
            )

            payload_hex = can_msg.data.hex()
            log_line = f"Sent CAN message: ID={hex(message.frame_id)}, Data={payload_hex}"
            _append_to_output(log_line)

            # Print the logged message to mirror previous console output.
            print(log_line)
        except Exception as e:
            print(f"Error encoding message: {e}")

        # Wait before sending the next message
        time.sleep(10)  # Adjust this for the frequency of sending messages

def random_can():
    while True:
        # Iterate over every message in the DBC file
        for message in db.messages:
            data = {}
            # Generate random value for each signal in the message
            for signal in message.signals:
                # Use the defined minimum/maximum or default to a full range based on the signal's length
                min_val = signal.minimum if signal.minimum is not None else 0
                max_val = signal.maximum if signal.maximum is not None else (2**signal.length - 1)
                data[signal.name] = random.randint(int(min_val), int(max_val))
            
            # Try to encode and send the message with the generated random data
            try:
                encoded_data = message.encode(data)
                can_msg = can.Message(
                    arbitration_id=message.frame_id,
                    data=encoded_data,
                    is_extended_id=False,
                )
                payload_hex = can_msg.data.hex()
                log_line = f"Sent CAN message: ID={hex(message.frame_id)}, Data={payload_hex}"
                _append_to_output(log_line)
                print(log_line)
            except Exception as e:
                print(f"Error encoding or sending message {message.name}: {e}")
        time.sleep(1)


if __name__ == "__main__":
    # random_can_single()
    random_can()