import os
import time

import numpy as np
import pandas as pd 
import json
import multiprocessing as mp
from multiprocessing import Process
# from multiprocessing.sharedctypes import Any
from dataclasses import dataclass, fields
from transitions import Machine


from gps_msgs.msg import GPSFix
from novatel_gps_msgs.msg import Inspvax

import rclpy
from rclpy.node import Node 

import cantools
import can
import ctypes

from ctypes import *
from sys import argv

# from ACC import * #to be fixed to avoid circular imports
# from scoring import * #to be fixed to avoid circular imports
# from callerC import * #to be fixed to avoid circular imports

# # from control_controller import *
# from control_controller_28Oct_v1 import * #to be fixed to avoid circular imports

# from sock_hmi_class import HMIServer
# from hypervisor import Hypervisor

from can_com_v2 import CAN_Rx, ce_rx_tx, sc_rx_tx


"""
Safety Code v0 Year 4 to 5

This code is part of the Buckeye AutoDrive Safety System.
Authors: Safety Team Year 3, 4. Reviewed by Safety Team Year 4 and 5 (Cristian Bautista)
Special thanks to Vishnu Renganathan. PhD.

This code is responsible for monitoring the safety of the
vehicle and taking appropriate ations

It monitors the following:
- Safety inspections 
- Doors status, Seatbelts status, motors status, steering status

It takes the following actions:
- Allows manual takeover if an issue is detected
- Communicates with the HMI to engage/disengage the Autonomous mode
- Comunicates to the vehicle to take actions (stop, slow down, steer, accelerate)
- Logs all the safety events
- CAN sending and receiving
- CAN scoring

Last updated: August 2025

To do:

"""
## -- CAN activation condition IDs -- ##

HS_184 = 388 # Steering wheel driver applied torque and message age
HS_3E9 = 1001 #Vehicle Speed
CE_170 = 368 #Automatic Braking message age
HS_C9 = 201 #Engine Check
HS_1E5 = 485 #Steering Wheel Angle (<10 deg)
HS_F1 = 241 #Pedal brake status
HS_230 = 560 #Park Brake Status
HS_12A = 298 #Driver searbelt and doors swithches

## -- Break Activation --##
# CE_170 = 368 #Automatic Braking message age

## -- Brake Release -- ##
# HS_F1 = 241 #Pedal brake status
# HS_3E9 = 1001 #Vehicle Speed

## -- STEERING Activation -- ##
# HS_1E5 = 485 #Steering Wheel Angle (<10 deg)
# HS_184 = 388 # Steering wheel driver applied torque and message age
HS_183 = 387 #Available for control

## -- Propulsion Activation -- ##
# Check torque AuthActiv == True

# Active mode
# Manual Takeover
# HS_184 < 4.5 NM
# HS_F1 == false
# HS_C9 < 1%
# HS_230 == released
# HMI Disengaged

# Comunication Fault Check
HS_C1 = 193
CE_C1 = 193
# HS_C9 = 201
# HS_F1
# HS_183
# HS_184
# Check software still alive.

HS_FILTERS = [
    {"can_id": HS_184, "can_mask": 0x7FF},
    {"can_id": HS_3E9, "can_mask": 0x7FF},
    {"can_id": HS_C9, "can_mask": 0x7FF},
    {"can_id": HS_1E5, "can_mask": 0x7FF},
    {"can_id": HS_F1, "can_mask": 0x7FF},
    {"can_id": HS_230, "can_mask": 0x7FF},
    {"can_id": HS_12A, "can_mask": 0x7FF},
    {"can_id": HS_183, "can_mask": 0x7FF},
    {"can_id": HS_C1, "can_mask": 0x7FF},
]

CE_FILTERS = [
    {"can_id": CE_170, "can_mask": 0x7FF},
    {"can_id": CE_C1, "can_mask": 0x7FF},
]

# -- State Machine -- ##
STATE_0 = 0
STATE_STARTUP = 1
STATE_PASSIVE_MODE = 2
STATE_ACTIVATION_CONDITION = 3
STATE_BRAKE_ACTIVATION = 4
STATE_WAIT_BRAKE_RELEASE = 5
STATE_STEER_ACTIVATION = 6
STATE_PROPULSION_ACTIVATION = 7
STATE_AV = 8
STATE_DEACTIVATION = 9
STATE_ACTIVATION_FAILURE = 10

## -- State Machine -- ##
states = [
    'STATE0',
    {'name':'STARTUP','on_enter':['starting_up']},
    {'name':'PASSIVE_MODE','on_enter':['passive_mode']},
    {'name':'ACTIVATION_CONDITION','on_enter':['activation_condition_check']},
    {'name':'BRAKE_ACTIVATION','on_enter':['brake_activation']},
    {'name':'WAIT_BRAKE_RELEASE','on_enter':['brake_release']},
    {'name':'STEER_ACTIVATION','on_enter':['steer_activation']},
    {'name':'PROPULSION_ACTIVATION','on_enter':['propulsion_activation']},
    {'name':'AV','on_enter':['av']},
    {'name':'DEACTIVATION','on_enter':['deactivation']},
    {'name':'ACTIVATION_FAILURE','on_enter':['activation_failure']}
]
transitions = [
    # -- Normal Operation Transitions -- #
    {'trigger': 'INIT', 'source': 'STATE0', 'dest': 'STARTUP'},
    {'trigger': 'STARTED_UP', 'source': 'STARTUP', 'dest': 'PASSIVE_MODE'},
    {'trigger': 'HMI_ENGAGED', 'source': 'PASSIVE_MODE', 'dest': 'ACTIVATION_CONDITION'},
    {'trigger': 'CONDITIONS_MET', 'source': 'ACTIVATION_CONDITION', 'dest': 'BRAKE_ACTIVATION'},
    {'trigger': 'BRAKE_ACTIVATED', 'source': 'BRAKE_ACTIVATION', 'dest': 'WAIT_BRAKE_RELEASE'},
    {'trigger': 'BRAKE_RELEASED', 'source': 'WAIT_BRAKE_RELEASE', 'dest': 'STEER_ACTIVATION'},
    {'trigger': 'STEER_ACTIVATED', 'source': 'STEER_ACTIVATION', 'dest': 'PROPULSION_ACTIVATION'},
    {'trigger': 'PROPULSION_ACTIVATED', 'source': 'PROPULSION_ACTIVATION', 'dest': 'AV'},
    {'trigger': 'MANUAL_TAKEOVER', 'source': 'AV', 'dest': 'DEACTIVATION'},
    {'trigger': 'COMMUNICATION_FAULT', 'source': 'AV', 'dest': 'DEACTIVATION'},
    {'trigger': 'SCENARIO_COMPLETED', 'source': 'AV', 'dest': 'DEACTIVATION'},
    {'trigger': 'DEACTIVATED', 'source': 'DEACTIVATION', 'dest': 'PASSIVE_MODE'},

    # -- Conditions to activation failure -- #
    {'trigger': 'CONDITIONS_NOT_MET', 'source': ['ACTIVATION_CONDITION', 'BRAKE_ACTIVATION','WAIT_BRAKE_RELEASE', 'STEER_ACTIVATION','PROPULSION_ACTIVATION'], 'dest': 'ACTIVATION_FAILURE'},
    {'trigger': 'TIMEOUT', 'source': 'WAIT_BRAKE_RELEASE', 'dest': 'ACTIVATION_FAILURE'},
    {'trigger': 'VEHICLE_MOVED', 'source': 'WAIT_BRAKE_RELEASE', 'dest': 'ACTIVATION_FAILURE'},
    {'trigger': 'RESET', 'source': 'ACTIVATION_FAILURE', 'dest': 'PASSIVE_MODE'},
]

## --Shared Memory Data Classes --##

@dataclass
class Steer:
    steer_val: mp.Value
    req_act_val: mp.Value
    steer_event : mp.Event()
    lock_steer: mp.Lock

@dataclass
class Brake:
    autbrktp_val: mp.Value 
    AccAct_val: mp.Value
    ACCaccl_val: mp.Value 
    lock_brake: mp.Lock

@dataclass
class Accel:
    AxlTrq_val: mp.Value
    ACC_Type_val: mp.Value
    ACCATC_DrvAstdGoSt_val: mp.Value
    ACCATC_SplREngInpR_val: mp.Value
    Prop_ACC_Act_val: mp.Value
    lock_accel: mp.Lock

@dataclass
class Avstate:
    global_auto_stat_val: mp.Value
    global_auto_stat_var: mp.Value
    steer_ctrl_val: mp.Value
    fric_brake_val: mp.Value
    prop_ctrl_val: mp.Value
    lock_avstate: mp.Lock

@dataclass
class cruise_switch:
    CruiseSwitch_speed_limiter_Status_val: mp.Value
    CruiseSecondry_Switch_Status_val: mp.Value
    lock_cruise_switch: mp.Lock

@dataclass
class AVstatus:
    can_event : mp.Event
    fsm_state : mp.Value
    fsm_lock : mp.Lock = mp.Lock()
    

@dataclass
class Flags:
    hmi_engaged : mp.Event

    act_con_met : mp.Event
    act_con_not_met : mp.Event

    can_warmed : mp.Event

    brake_activated : mp.Event
    brake_act_failed : mp.Event

    brake_released : mp.Event
    brake_rel_failed : mp.Event

    steer_activated : mp.Event
    steer_act_failed : mp.Event
    
    prop_activated : mp.Event
    prop_act_failed : mp.Event
    
    av_mode : mp.Event
    mto : mp.Event
    comm_fault : mp.Event
    scenario_completed : mp.Event
    flag_lock : mp.Lock


@dataclass
class Shared:
    steer: Steer
    brake: Brake
    accel: Accel
    avstate: Avstate
    cruise: cruise_switch
    avstatus: AVstatus
    flags: Flags
    shutdown: mp.Event  # To be implemented

def init_shared():

    ##--Steering --##
    steer_val = mp.Value('i', 0) # value between -540 to 540 ()
    req_act_val = mp.Value('i', 0)
    steer_event = mp.Event()
    lock_steer = mp.Lock()

    ##--Brake --##
    autbrktp_val = mp.Value('i', 1) # 6 posible states (0 to 5) see CAN DB (Initially 1: Not braking)
    AccAct_val = mp.Value('i', 0) # To be understood
    ACCaccl_val = mp.Value('d', 0) #ADC_CE: Written by the Controller multiprocess (-3 to 0 m/s^2)
    lock_brake = mp.Lock()

    ##--Acceleration --##
    AxlTrq_val = mp.Value('i', 0)
    ACC_Type_val = mp.Value('i', 0)
    ACCATC_DrvAstdGoSt_val = mp.Value('i', 0)
    ACCATC_SplREngInpR_val = mp.Value('i', 0)
    Prop_ACC_Act_val = mp.Value('i', 0)
    lock_accel = mp.Lock()

    ##--AVState --##
    global_auto_stat_val = mp.Value('i', 1) # to be evaluated
    global_auto_stat_var = mp.Value('b', True)
    steer_ctrl_val = mp.Value('i', 0)
    fric_brake_val = mp.Value('i', 0)
    prop_ctrl_val = mp.Value('i', 0)
    lock_avstate = mp.Lock()

    ##--Cruise Switch --##
    CruiseSwitch_speed_limiter_Status_val = mp.Value('i', 0)
    CruiseSecondry_Switch_Status_val = mp.Value('i', 0)
    lock_cruise_switch = mp.Lock()

    ##--AV Status --##
    can_event = mp.Event() ; 
    fsm_state = mp.Value('i', 0)
    fsm_lock = mp.Lock()

    ##--Flags --##
    hmi_engaged = mp.Event()

    act_con_met = mp.Event() #Verify if works
    act_con_not_met = mp.Event()

    can_warmed = mp.Event()

    brake_activated = mp.Event()
    brake_act_failed = mp.Event()

    brake_released = mp.Event()
    brake_rel_failed = mp.Event()

    steer_activated = mp.Event()
    steer_act_failed = mp.Event()

    prop_activated = mp.Event()
    prop_act_failed = mp.Event()

    av_mode = mp.Event()
    mto = mp.Event()
    comm_fault = mp.Event()
    scenario_completed = mp.Event()

    flag_lock = mp.Lock()



    shutdown = mp.Event()

    return Shared(
        steer=Steer(steer_val, req_act_val, steer_event, lock_steer),
        brake=Brake(autbrktp_val, AccAct_val, ACCaccl_val, lock_brake),
        accel=Accel(AxlTrq_val, ACC_Type_val, ACCATC_DrvAstdGoSt_val, ACCATC_SplREngInpR_val, Prop_ACC_Act_val, lock_accel),
        avstate=Avstate(global_auto_stat_val, global_auto_stat_var, steer_ctrl_val, fric_brake_val, prop_ctrl_val, lock_avstate),
        cruise=cruise_switch(CruiseSwitch_speed_limiter_Status_val, CruiseSecondry_Switch_Status_val, lock_cruise_switch),
        avstatus=AVstatus(can_event, fsm_state, fsm_lock),
        flags=Flags(hmi_engaged, act_con_met, act_con_not_met, can_warmed, brake_activated, brake_act_failed, brake_released, 
                    brake_rel_failed, steer_activated, steer_act_failed, prop_activated, prop_act_failed, av_mode, mto, comm_fault,
                    scenario_completed, flag_lock),
        shutdown=shutdown
    )

# -- State Machine Execution -- #
class Vehicle:
    def __init__(self):
        self.shared = None
        self.hs_p = None 
        self.ce_p = None
        self.sc_p = None
        self.s = self.shared

    def starting_up(self):
               
        print("Starting up...")
        self.shared = init_shared()
        print("Shared memory initialized.")
        self.s = self.shared
        self.s.avstatus.can_event.set()
        

        self.hs_p = mp.Process(target=CAN_Rx, args=(self.shared,), name='CAN_Rx')
        # self.ce_p = mp.Process(target=ce_rx_tx, args=(self.shared, CE_FILTERS), name='CE_CAN')
        # self.sc_p = mp.Process(target=sc_rx_tx, args=(self.shared,), name='SC_CAN')
        self.hs_p.start()#; self.ce_p.start(); self.sc_p.start()
        time.sleep(1)

        fsm_lock = self.s.avstatus.fsm_lock
        with fsm_lock:
            self.s.avstatus.fsm_state.value = STATE_STARTUP

    def passive_mode(self):
        ans = ""
        os.system('clear')
        hmi_engage = car.shared.flags.hmi_engaged
        while not hmi_engage.is_set():
            print("[HMI]: Disengaged. Waiting...")
            hmi_engage.wait(timeout=2)

            ans = input('Press [y] for manual engagement / Enter for waiting: ')

            if ans.lower() == "y":
                hmi_engage.set()
                break
            time.sleep(5)

        if ans.lower() == "y":
            print('[AV]: Engaging in progress (keyboard)')
        else:
            print('[AV]: Engaging in progress')
            print('[HMI]: Engaged')
        time.sleep(1)

    def activation_condition_check(self):
        print('[S]: Activation Conditions Check')

    def brake_activation(self):
        os.system('clear')
        print('[S]: Brake activation')

    def brake_release(self):
        os.system('clear')
        print('[S]: Brake release')
    
    def steer_activation(self):
        os.system('clear')
        print('[S]: Steer activation')

    def propulsion_activation(self):
        os.system('clear')
        print('[S]: Propulsion Activation')
    
    def av(self):
        os.system('clear')
        print('[S]: Autonomous Mode On ... Enjoy :)')

    def deactivation(self):
        #os.system('clear')
        reset_flags(car.shared)
        print('[S]: Deactivation')
        """
        it should send to the CAN_Tx script to reset everything
        """
        time.sleep(2)
    def activation_failure(self):

        ## -- Flags Reset -- ##
        reset_flags(car.shared)      
        print('[S]: Activation Failure')
        time.sleep(1)




def build_state_machine(car: Vehicle):
    m = Machine(
        model=car,
        states=states,
        initial='STATE0',
        transitions=transitions,
        auto_transitions=False)
    
def reset_flags(shared):
    
    # for f in fields(shared.flags):
    #     event = getattr(shared.flags, f.name)
    #     if isinstance(event, mp.Event):
    #         event.clear()

    shared.flags.hmi_engaged.clear() #HMI in the Bolt should switch to Disengage

    shared.flags.act_con_met.clear()
    shared.flags.act_con_not_met.clear()

    shared.flags.brake_activated.clear()
    shared.flags.brake_act_failed.clear()

    shared.flags.brake_released.clear()
    shared.flags.brake_rel_failed.clear()

    shared.flags.steer_activated.clear()
    shared.flags.steer_act_failed.clear()

    shared.flags.prop_activated.clear()
    shared.flags.prop_act_failed.clear()

    shared.flags.av_mode.clear()
    shared.flags.mto.clear()
    shared.flags.comm_fault.clear()
    shared.flags.scenario_completed.clear()

if __name__ == "__main__":
    mp.set_start_method('spawn', force=True)
    car = Vehicle()
    print("Init")
    build_state_machine(car)
    
    car.INIT() # [FSM]: State0 -> Startup
    s = car.shared

    fsm_lock = s.avstatus.fsm_lock

    can_warmed = s.flags.can_warmed

    act_con_met = s.flags.act_con_met
    act_con_not_met = s.flags.act_con_not_met

    brake_activated = s.flags.brake_activated
    brake_act_failed = car.shared.flags.brake_act_failed

    brake_released = car.shared.flags.brake_released
    brake_rel_failed = car.shared.flags.brake_rel_failed

    steer_activated = car.shared.flags.steer_activated
    steer_act_failed = car.shared.flags.steer_act_failed

    prop_activated = car.shared.flags.prop_activated
    prop_act_failed = car.shared.flags.prop_act_failed

    av_mode = car.shared.flags.av_mode
    mto = car.shared.flags.mto
    comm_fault = car.shared.flags.comm_fault
    scenario_completed = car.shared.flags.scenario_completed



    while True: #Another condition should be consider for a clean shut down
        t0 = time.time()
        with fsm_lock:
            fsm_state = s.avstatus.fsm_state.value
        
        if fsm_state == STATE_STARTUP and s.avstatus.can_event.is_set():
            if can_warmed.wait(timeout=3.0): #timer to be tested 
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_PASSIVE_MODE
                car.STARTED_UP() #FSM: statup -> passive mode (Launch passive_mode())

        elif fsm_state == STATE_PASSIVE_MODE and s.flags.hmi_engaged.is_set():
            time.sleep(1.5) #Time to trigger an activation failure
            with fsm_lock:
                s.avstatus.fsm_state.value = STATE_ACTIVATION_CONDITION
            car.HMI_ENGAGED() #FSM: Passive Mode -> Activation Condition Check (activation_condition_check())

        elif fsm_state == STATE_ACTIVATION_CONDITION: #To be tested if reacts as expected (Fast)
            if act_con_met.is_set():
                print('Activation Condition Check: OK')
                time.sleep(1)
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_BRAKE_ACTIVATION
                car.CONDITIONS_MET()
            elif act_con_not_met.is_set():
                print('Conditions not met')
                time.sleep(1)
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_ACTIVATION_FAILURE
                car.CONDITIONS_NOT_MET()
            
        elif fsm_state == STATE_BRAKE_ACTIVATION:

            if brake_activated.is_set():
                print('Brake Activation Completed')
                # time.sleep(4)
                # print('debug 1')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_WAIT_BRAKE_RELEASE
                car.BRAKE_ACTIVATED()
            elif brake_act_failed.is_set():
                print('Brake Activation Failed')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_ACTIVATION_FAILURE
                car.CONDITIONS_NOT_MET()

        elif fsm_state == STATE_WAIT_BRAKE_RELEASE:

            if brake_released.is_set():
                print('Brake Released')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_STEER_ACTIVATION
                car.BRAKE_RELEASED()
            elif brake_rel_failed.is_set():
                print('Brake released failed')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_ACTIVATION_FAILURE
                car.CONDITIONS_NOT_MET()
            time.sleep(1)

        elif fsm_state == STATE_STEER_ACTIVATION:

            if steer_activated.is_set():
                print('Steer Activated')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_PROPULSION_ACTIVATION
                car.STEER_ACTIVATED()

            elif steer_act_failed.is_set():
                print('Steer Activation failed')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_ACTIVATION_FAILURE
                car.CONDITIONS_NOT_MET()
            time.sleep(1)

        elif fsm_state == STATE_PROPULSION_ACTIVATION:

            if prop_activated.is_set():
                print('Propulsion Activated')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_AV
                car.PROPULSION_ACTIVATED()

            elif prop_act_failed.is_set():
                print('Propulsion Activation failed')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_ACTIVATION_FAILURE
                car.CONDITIONS_NOT_MET()
        
        elif fsm_state == STATE_AV:

            if mto.is_set():
                print('Manual Takeover performed')
                time.sleep(2)
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_DEACTIVATION
                car.MANUAL_TAKEOVER()
            
            elif comm_fault.is_set():
                print('Comminication fault with CAN bus')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_DEACTIVATION
                car.COMMUNICATION_FAULT()
            
            elif scenario_completed.is_set():
                print('Scenario Completed')
                with fsm_lock:
                    s.avstatus.fsm_state.value = STATE_DEACTIVATION
                car.SCENARIO_COMPLETED()

        elif fsm_state == STATE_DEACTIVATION:
            print('Deactivating...')
            with fsm_lock:
                s.avstatus.fsm_state.value = STATE_PASSIVE_MODE
            car.DEACTIVATED()

        elif fsm_state == STATE_ACTIVATION_FAILURE:
            with fsm_lock:
                s.avstatus.fsm_state.value = STATE_PASSIVE_MODE
            car.RESET()

        time.sleep(max(0, 0.01 - (time.time() - t0)))
            
            









    
"""

To be implemented to close all processes safely
    try:
        p_steer.join()
    except KeyboardInterrupt:
        print("Shutting down...")
        shared.shutdown.set()
        p_steer.terminate()
        p_steer.join()
        print("Shutdown complete.")
"""
