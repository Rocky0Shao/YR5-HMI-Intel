import can
import cantools
import time
import threading
 
# Load the DBC file
dbc_path = "scoring_can.dbc"
dbc = cantools.database.load_file(dbc_path)
 
# Find the message that contains the GlobalAutonomyStatus signal
gas_message_name = 'AVState'  # Assuming the message name from your excerpt
gas_message = dbc.get_message_by_name(gas_message_name)


 
# Blue Light message setup
bl_message_name = 'AVLight'  # Assuming the message name from your excerpt
bl_message = dbc.get_message_by_name(bl_message_name)
 
# Setup the CAN bus
bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000)
 
def send_global_autonomy_status(status_value, Rolling_Count):
    # Construct the data for the message including the GlobalAutonomyStatus signal
    data = gas_message.encode({'GlobalAutonomyStatus': status_value, 'Rolling_Count': Rolling_Count, 'SteeringCtrlActive': 0, 'FrictionBrakeCtrlActive': 1, 'PropulsionCtrlActive': 0})
    # Create the CAN message
    can_message = can.Message(arbitration_id=gas_message.frame_id, data=data, is_extended_id=False)
    # Send the message
    bus.send(can_message)
 
def cycle_global_autonomy_status():
    Rolling_Count = 0
    while True:  # Loop forever, or adjust this as needed for your testing
        for status_value in [0, 1, 2]:  # Cycle through the signal values
            print(f"It is being sent AVState with GlobalAutonomyStatus={status_value}")
            start = time.monotonic()
            while (time.monotonic()-start) < 10:
                if Rolling_Count != 3:
                    Rolling_Count += 1
                else:
                    Rolling_Count = 0
                send_global_autonomy_status(status_value, Rolling_Count)
                time.sleep(1.0)  # Wait for 15 seconds before the next value

def send_blue_light_status(status_value, Rolling_Count):
    # Construct the data for the message including the AVLightStatus and AVLightColor signal
    data = bl_message.encode({'Rolling_Count': Rolling_Count, 'AVLightStatus': status_value, 'AVLightColor': 3})
    # Create the CAN message
    can_message = can.Message(arbitration_id=bl_message.frame_id, data=data, is_extended_id=False)
    # Send the message
    bus.send(can_message)
 
def cycle_blue_light_status():
    Rolling_Count = 0
    while True:  # Loop forever, or adjust this as needed for your testing
        for status_value in [0, 1, 2]:  # Cycle through the signal values
            print(f"It is being sent AVLight with AVLightStatus={status_value}")
            start = time.monotonic()
            while (time.monotonic()-start) < 10:
                if Rolling_Count != 3:
                    Rolling_Count += 1
                else:
                    Rolling_Count = 0
                send_blue_light_status(status_value, Rolling_Count)
                time.sleep(1.0)  # Wait for 15 seconds before the next value



threads = []

gas_thread = threading.Thread(target=cycle_global_autonomy_status, args=(), kwargs={})

bl_thread = threading.Thread(target=cycle_blue_light_status, args=(), kwargs={})

threads.append(gas_thread)

threads.append(bl_thread)

# Start each thread
for t in threads:
    t.start()

# Wait for all threads to finish
for t in threads:
    t.join()

