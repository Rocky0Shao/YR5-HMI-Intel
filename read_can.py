import atexit
import can
import socket
import cantools
from pprint import pprint

import send_can_protobuf

scoring_db = cantools.database.load_file("scoring_can.dbc")




def shutdown_bus(bus):
    bus.shutdown()



can_interface = "vcan0"

bus = can.interface.Bus(can_interface, bustype='socketcan', bitrate=500000)

atexit.register(shutdown_bus, bus)


gas_message_name = 'AVState'  # Assuming the message name from your excerpt
gas_message = scoring_db.get_message_by_name(gas_message_name)


 
# Blue Light message setup
bl_message_name = 'AVLight'  # Assuming the message name from your excerpt
bl_message = scoring_db.get_message_by_name(bl_message_name)

for msg in bus:
    print("raw data: ", msg.data)
    print("decoded data: ", scoring_db.decode_message(msg.arbitration_id, msg.data, decode_choices=False))

    if msg.arbitration_id == gas_message.frame_id:
        print("Gas message detected")
        d = scoring_db.decode_message(msg.arbitration_id, msg.data, decode_choices=False) #decoded av state
        serialized_av_state = send_can_protobuf.build_av_state_message(d['Rolling_Count'], d['GlobalAutonomyStatus'], d['SteeringCtrlActive'], d['FrictionBrakeCtrlActive'], d['PropulsionCtrlActive'])
        send_can_protobuf.send_payload(serialized_av_state)

    if msg.arbitration_id == bl_message.frame_id:
        print("BL message detected")
        d = scoring_db.decode_message(msg.arbitration_id, msg.data, decode_choices=False) #decoded av light
        serialized_av_light = send_can_protobuf.build_av_light_message(d['Rolling_Count'], d['AVLightStatus'], d['AVLightColor'])
        send_can_protobuf.send_payload(serialized_av_light)


def initiate_connection():
    """
    Starts the connection with the HMI container
    """
    print("nothing implemented yet")