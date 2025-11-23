import HMI_RX_CAN_pb2 as can_pb2
import sys

import socket
import struct


# Get an IP (replace with IP of the machine to commmunicate)
ip = "127.0.0.1"

port = 5002



def build_av_light_message(rolling_count, av_light_status, av_light_color):
    #building a message and serialize it
    av_light = can_pb2.AVLight()
    av_light.Rolling_Count = rolling_count
    av_light.AVLightStatus = av_light_status
    av_light.AVLightColor = av_light_color
    return av_light


def build_av_state_message(rolling_count, global_autonomy_status, steering_ctrl_active, friction_brake_ctrl_active, propulsion_ctrl_active):
    av_state = can_pb2.AVState()
    av_state.Rolling_Count = rolling_count
    av_state.GlobalAutonomyStatus = global_autonomy_status
    av_state.SteeringCtrlActive = steering_ctrl_active
    av_state.FrictionBrakeCtrlActive = friction_brake_ctrl_active
    av_state.PropulsionCtrlActive = propulsion_ctrl_active




    return av_state

def build_message_wrapper(type_str, msg):

    message_wrapper = can_pb2.MessageWrapper()

    if type_str == "av_light":
        message_wrapper.av_light.CopyFrom(msg)
        
        
    elif type_str == "av_state":
        message_wrapper.av_state.CopyFrom(msg)


    return message_wrapper.SerializeToString()

try: 
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.settimeout(2.0)
    s.connect((ip, port))
    s.settimeout(0.5)
except socket.error as err:
    print("Error with socket creation")


#send messages to socket
def send_payload(payload):

    frame = struct.pack(">I", len(payload)) + payload
    s.sendall(frame)

    #s.send("testframe".encode())


"""
syntax = "proto3";

message AVLight {
    int32 Rolling_Count = 1;
    int32 AVLightStatus = 2;
    int32 AVLightColor = 3;
}
message AVState {
    int32 Rolling_Count = 1;
    int32 GlobalAutonomyStatus = 2;
    int32 SteeringCtrlActive = 3;
    int32 FrictionBrakeCtrlActive = 4;
    int32 PropulsionCtrlActive = 5;
}


/**BO_ 16 AVLight: 1 Vector__XXX
 * SG_ Rolling_Count : 7|2@0+ (1,0) [0|3] "" DAQ_SC,BlueLight_SC
 * SG_ AVLightStatus : 5|2@0+ (1,0) [0|3] "" DAQ_SC,BlueLight_SC
 * SG_ AVLightColor : 3|3@0+ (1,0) [0|5] "" Vector__XXX

 *BO_ 17 AVState: 1 Vector__XXX
 * SG_ Rolling_Count : 7|2@0+ (1,0) [0|3] "" DAQ_SC,BlueLight_SC
 * SG_ GlobalAutonomyStatus : 5|2@0+ (1,0) [0|2] "" DAQ_SC,BlueLight_SC
 * SG_ SteeringCtrlActive : 3|1@0+ (1,0) [0|1] "" DAQ_SC
 * SG_ FrictionBrakeCtrlActive : 2|1@0+ (1,0) [0|1] "" DAQ_SC
 * SG_ PropulsionCtrlActive : 1|1@0+ (1,0) [0|1] "" DAQ_SC
 */

"""
