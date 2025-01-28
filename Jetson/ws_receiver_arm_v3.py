import socket
import hashlib
import asyncio
import json
import numpy as np
import sys
import os
from colorama import init as colorama_init
from colorama import Cursor
import moteus
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library
# import logging

# logging.basicConfig(level=logging.DEBUG)

# ========================= GRIPPER ===========================
# Control table addresses
ADDR_PRO_TORQUE_ENABLE = 64
ADDR_PRO_LED = 65  # Address to control LED
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_PRESENT_POSITION = 132
ADDR_PRO_PRESENT_TEMPERATURE = 146
ADDR_PRO_GOAL_PWM = 100  # Address for setting PWM
ADDR_PRO_TORQUE_STATUS = 64  # Torque enable status address (read)

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
DXL_ID = 1  # ID of your Dynamixel
BAUDRATE = 57600
# DEVICENAME = '/dev/ttyACM0'  # Change to the port connected to your Dynamixel
DEVICENAME = ''

# Torque settings
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
LED_ON = 1
LED_OFF = 0

# Position settings
OPEN_POSITION = 1178
CLOSED_POSITION = 3145
POSITION_THRESHOLD = 20  # Threshold to check if gripper is already in position

cur_gripper_state = -1  # -1 means open (value -0.022 in sim), +1 means close (value 0.015 in sim)
last_gripper_temperature = -1.0

def is_torque_enabled():
    """Check if the servo torque is already enabled."""
    torque_status, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(
        port_handler, DXL_ID, ADDR_PRO_TORQUE_ENABLE
    )
    if dxl_comm_result != COMM_SUCCESS:
        raise Exception(f"Failed to read torque status: {packet_handler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error:
        raise Exception(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")
    return torque_status == TORQUE_ENABLE


def set_led(state: int):
    """Control the LED state (on/off)."""
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_PRO_LED, state)
    if dxl_comm_result != COMM_SUCCESS:
        raise Exception(f"Failed to set LED: {packet_handler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error:
        raise Exception(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")
    print(f"LED {'ON' if state == LED_ON else 'OFF'}")


def control_gripper(position: str, pwm: int):
    
        # Get current position
        current_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
            port_handler, DXL_ID, ADDR_PRO_PRESENT_POSITION
        )
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to read position: {packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error:
            raise Exception(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

        # Read the present temperature (always read temperature regardless of state)
        dxl_present_temperature, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(
            port_handler, DXL_ID, ADDR_PRO_PRESENT_TEMPERATURE
        )
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to read temperature: {packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error:
            raise Exception(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

        # print(f"Current temperature: {dxl_present_temperature}°C")

        # If "open" command and already open, skip additional actions
        if position == "open" and abs(current_position - OPEN_POSITION) < POSITION_THRESHOLD:
            print("Gripper is already open. Skipping command.")
            return dxl_present_temperature  # Still return the temperature

        # Enable Dynamixel Torque and turn LED on if not already enabled
        if not is_torque_enabled():
            packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            set_led(LED_ON)
            print("Torque enabled.")

        # Set gripping force (PWM)
        dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, DXL_ID, ADDR_PRO_GOAL_PWM, abs(pwm))
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to set PWM: {packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error:
            raise Exception(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

        # Set the goal position
        goal_position = OPEN_POSITION if position == "open" else CLOSED_POSITION
        packet_handler.write4ByteTxRx(port_handler, DXL_ID, ADDR_PRO_GOAL_POSITION, goal_position)

        # Handle "close" command differently
        if position == "close":
            print("Gripper closing. Not waiting for exact position.")
            # Immediately return after issuing the command
        else:
            # Wait until the servo reaches the open position
            while True:
                dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
                    port_handler, DXL_ID, ADDR_PRO_PRESENT_POSITION
                )
                if dxl_comm_result != COMM_SUCCESS:
                    raise Exception(f"Failed to read position: {packet_handler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error:
                    raise Exception(f"Dynamixel error: {packet_handler.getRxPacketError(dxl_error)}")

                if abs(goal_position - dxl_present_position) < POSITION_THRESHOLD:
                    print("Gripper reached open position.")
                    break
                time.sleep(0.1)

            # Disable torque and turn LED off after opening
            packet_handler.write1ByteTxRx(port_handler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            set_led(LED_OFF)
            print("Torque disabled after opening gripper")

        return dxl_present_temperature


def set_gripper_state(state):
    global last_gripper_temperature
    position, pwm = None, None
    if state == -1:
        position = "open"  # "open" or "close"
        pwm = 500  # Adjust PWM value (gripping force) max is 885
    elif state == 1:
        position = "close"  # "open" or "close"
        pwm = 200  # Adjust PWM value (gripping force) max is 885
    else:
        raise NotImplementedError(f'Gripper state == {state} is not implemented!')
    try:
        # Open port
        if not port_handler.openPort():
            raise Exception("Failed to open the port")
        port_handler.setBaudRate(BAUDRATE)
        
        temperature = control_gripper(position, pwm)
        if temperature is not None:
            # print(f"The servo's temperature is: {temperature}°C")
            last_gripper_temperature = float(temperature)
        else:
            last_gripper_temperature = -1.0
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close port
        port_handler.closePort()


use_gripper = True
if use_gripper:
    DEVICENAME = '/dev/ttyACM1'  # Change to the port connected to your Dynamixel
    # Initialize PortHandler and PacketHandler
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
# ========================= UDP ===========================

# # Initialize simulated joint state
# joint_positions = [0.0] * 8
# joint_temperatures = [25.0] * 8  # Start with a default temperature

def verify_checksum(data, checksum):
    """Verify checksum of received data."""
    calculated_checksum = hashlib.md5(data.encode()).hexdigest()
    return calculated_checksum == checksum

def process_command(command):
    """Process a command and update the robot state."""
    # global joint_positions
    if command.startswith("SET_JOINT_POSITIONS"):
        try:
            # Extract joint positions
            _, *positions = command.split()
            joint_positions = list(map(float, positions))
            # print(f"Updated joint positions: {joint_positions}")

            poses_data = None  # ignore
            joints_data = {(i+1): joint_positions[i] for i in range(len(joint_positions))}
            gripper_state = 1 if joints_data[8] > 0 else -1
            return joints_data, poses_data, gripper_state
        # except ValueError:
        #     print("Invalid joint positions received.")
        except Exception as e:
            print(f"Error receiving data: {e}")
    else:
        print(f"Unknown command: {command}")

def create_response(current_positions, temperatures):
    """Create a response message with current positions and temperatures."""
    positions_str = " ".join(f"{pos:.3f}" for pos in current_positions)
    temperatures_str = " ".join(f"{temp:.2f}" for temp in temperatures)
    response = f"CURRENT_STATE {positions_str} {temperatures_str}"
    checksum = hashlib.md5(response.encode()).hexdigest()
    return f"{response}|{checksum}"

sock = None
use_udp = True
if use_udp:
    # Configure the UDP server
    UDP_IP = "0.0.0.0"  # Listen on all interfaces
    UDP_PORT = 5005     # Port to listen on

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Server listening on {UDP_IP}:{UDP_PORT}")

path_to_json_file = '/home/skyentific/Downloads/poses_data.json'
velocity_limit = 1.0
# Initialize colorama to support ANSI codes on Windows
colorama_init()

received_data = None
empty_str = '                                                                                \n'*(9)
status_str = empty_str
joint_status_str = '                                                                                \n'*(7)

all_motor_ids = [1, 2, 3, 4, 5, 6, 7]
motor_pair_ids = [[1], [2, 3], [4, 5], [6, 7]]
motor_home_poses = {
    7: 0.030,
    6: -0.051,
    5: -0.237,
    4: 0.092,
    3: -0.427,
    2: 0.060,
    1: -0.377,
}
motor_ranges = {
    7: [-3.35, 2.70],
    6: [-3.44, 2.61],
    5: [-5.76, 6.76],
    4: [-6.88, 5.54],
    3: [-7.65, 4.30],
    2: [-4.58, 7.26],
    1: [-8.21, 2.11],
}
reduction_ratios = {
    7:  4.10,
    6:  4.10,
    5:  8.14286,
    4:  8.14286,
    3:  8.14286,
    2:  8.14286,
    1: 10.76190,
}
joint_signs = {k: 1 for k in all_motor_ids}
joint_signs[6] = -1

active_motor_ids = [1, 2, 3, 4, 5, 6, 7]
# active_motor_ids = [6, 7]

motors = {id: moteus.Controller(id=id) for id in active_motor_ids}
motor_temperatures = {id: 0.0 for id in active_motor_ids}
motor_range_limit_reached = {id: False for id in active_motor_ids}

cur_rel_motor_poses = {id: 0.0 for id in active_motor_ids}
cur_abs_motor_poses = {id: motor_home_poses[id] for id in active_motor_ids}
cur_abs_motor_poses_from_motors = {id: 0.0 for id in active_motor_ids}

# check motor pairs
active_motor_pair_ids = []
active_joint_ids = []
for pair in motor_pair_ids:
    if len(pair) == 1:
        if pair[0] in active_motor_ids:
            active_motor_pair_ids.append(pair)
            active_joint_ids.append(pair[0])
        continue
    p0, p1 = pair[0] in active_motor_ids, pair[1] in active_motor_ids
    if p0 or p1:
        assert p0 and p1, f'Error: the motor pair {pair} is not completed!' \
            f' Add motor {pair[0] if p1 else pair[1]} to active_motor_ids or remove motor {pair[0] if p0 else pair[1]}'
        active_motor_pair_ids.append(pair)
        active_joint_ids.extend(pair)
# joint_poses = np.zeros((len(joint_ids),))
joint_poses = {id: 0.0 for id in active_joint_ids}

results = ''

delay = 0.002
retry_delay = 0.002
keep_alive_period = 0.01
handler_period = 0.005
pos_min_delta = 0.005

# os.system('cls')  # anaconda prompt
os.system('clear')  # JETSON

async def motors_set_stop():
    global motors
    for c in motors.values():
        await c.set_stop()
    await asyncio.sleep(delay)

def read_json(file_path):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return None

def write_json(file_path, data):
    try:
        with open(file_path, 'w') as file:
            json.dump(data, file, indent=4)
    except Exception as e:
        print(f"An error occurred while writing to the file: {e}")

def read_commands_from_json():
    try:
        data = read_json(path_to_json_file)
        if data is not None:
            if "joints_rad" in data or "joints_deg" in data:
                if "joints_deg" in data:
                    joints_data = {int(k): np.deg2rad(v) for k, v in data["joints_deg"].items()}
                else:
                    joints_data = {int(k): v for k, v in data["joints_rad"].items()}
                poses_data = None  # ignore
            else:
                joints_data = None
                poses_data = {int(k): v for k, v in data["poses"].items()}

                # assert 'gripper' in data, 'There is no gripper state in the data!'
            if 'gripper' in data:
                gripper_state = 1 if data['gripper'] > 0 else -1
            else:
                gripper_state = None
        return joints_data, poses_data, gripper_state
    except Exception as e:
        print(f"Error in read_commands_from_json: {e}")
        raise Exception(e)

def read_commands_from_udp():
    global sock, joint_poses, motor_temperatures, cur_gripper_state, last_gripper_temperature
    new_state_received = False
    while not new_state_received:
        try:
            data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
            message = data.decode()
            command, checksum = message.rsplit("|", 1)  # Split data and checksum
            if verify_checksum(command, checksum):
                joints_data, poses_data, gripper_state = process_command(command)

                current_positions = [turn2rad(joint_poses[i]) if i in joint_poses else -1.0 for i in range(1, 7+1)] + [cur_gripper_state if cur_gripper_state is not None else 0.0]
                temperatures = [motor_temperatures[i] if i in motor_temperatures else -1.0  for i in range(1, 7+1)] + [last_gripper_temperature if last_gripper_temperature is not None else -1.0]
                response = create_response(current_positions, temperatures)
                sock.sendto(response.encode(), addr)
                # print(f"Sent state to {addr}: {response}")
                new_state_received = True
                return joints_data, poses_data, gripper_state
            else:
                print(f"Checksum mismatch! Corrupted data: {message}")
            new_state_received = False

        except Exception as e:
            print(f"Error receiving data: {e}")

def turn2rad(t):
    return t * 2 * np.pi

def rad2turn(r):
    return r / 2 / np.pi

def make_nice_str():
    global status_str, use_gripper, joint_status_str, cur_gripper_state, last_gripper_temperature, active_motor_pair_ids, cur_rel_motor_poses, cur_abs_motor_poses, cur_abs_motor_poses_from_motors, motor_range_limit_reached
    
    status_str = "Motor ID                          Positions                                        Temperature           Limit reached               \n"
    status_str+= "                (relative)        (absolute)        (absolute on the motor)                                                          \n"
    joint_status_str = ""
    if len(active_motor_pair_ids) > 0:
        for pair in active_motor_pair_ids:
            if len(pair) == 1:
                i = pair[0]
                status_str += f"    {i}            {cur_rel_motor_poses[i]:.4f}            {cur_abs_motor_poses[i]:.4f}              {cur_abs_motor_poses_from_motors[i]:.4f}                    {motor_temperatures[i]:.1f}               {motor_range_limit_reached[i]}   \n"
                joint_status_str += f"J{i}:    {turn2rad(joint_poses[i]):.4f}  rad    or    {np.rad2deg(turn2rad(joint_poses[i])):.4f}  deg                                                                 \n"
                continue
            i_0, i_1 = pair
            status_str += f"    {i_0}            {cur_rel_motor_poses[i_0]:.4f}            {cur_abs_motor_poses[i_0]:.4f}              {cur_abs_motor_poses_from_motors[i_0]:.4f}                    {motor_temperatures[i_0]:.1f}               {motor_range_limit_reached[i_0]}   \n"
            status_str += f"    {i_1}            {cur_rel_motor_poses[i_1]:.4f}            {cur_abs_motor_poses[i_1]:.4f}              {cur_abs_motor_poses_from_motors[i_1]:.4f}                    {motor_temperatures[i_1]:.1f}               {motor_range_limit_reached[i_1]}   \n"
            joint_status_str += f"J{i_0}:    {turn2rad(joint_poses[i_0]):.4f}  rad    or    {np.rad2deg(turn2rad(joint_poses[i_0])):.4f}  deg                                                                 \n"
            joint_status_str += f"J{i_1}:    {turn2rad(joint_poses[i_1]):.4f}  rad    or    {np.rad2deg(turn2rad(joint_poses[i_1])):.4f}  deg                                                                 \n"
    else:
        status_str = 'No active motors! \n'
    if use_gripper:
        joint_status_str += f"G :    {cur_gripper_state}      {'close' if cur_gripper_state > 0 else 'open'}      {last_gripper_temperature:.1f}°C"

def print_status():
    global status_str, joint_status_str

    sys.stdout.write(Cursor.POS(0, 0))
    make_nice_str()
    print(status_str + joint_status_str)
    # print(results)

async def safe_set_position(controller, position, velocity, velocity_limit, accel_limit, query):
    for attempt in range(3):  # Retry up to 3 times
        try:
            # print(f"Attempt {attempt + 1}: Setting position to {position}")
            return await controller.set_position(
                position=position,
                velocity=velocity,
                velocity_limit=velocity_limit,
                accel_limit=accel_limit,
                query=query
            )
        except RuntimeError as e:
            print(f"Attempt {attempt + 1}: Error setting position - {e}")
            await asyncio.sleep(retry_delay)  # Small delay before retrying
    raise RuntimeError("Failed to set position after multiple attempts")

async def handler():
    global use_udp, results, use_gripper, cur_gripper_state, velocity_limit, motors, active_motor_pair_ids, cur_rel_motor_poses, cur_abs_motor_poses, cur_abs_motor_poses_from_motors, motor_temperatures, motor_range_limit_reached, joint_poses, reduction_ratios
    while True:
        try:
            if use_udp:
                joints_data, poses_data, gripper_state = read_commands_from_udp()
            else:
                joints_data, poses_data, gripper_state = read_commands_from_json()

            # set positions
            #     gripper
            if use_gripper:
                assert gripper_state is not None, 'There is no gripper state in the data!'
                if cur_gripper_state != gripper_state:
                    cur_gripper_state = gripper_state
                    set_gripper_state(cur_gripper_state)

            #     motors
            for pair in active_motor_pair_ids:
                if len(pair) == 1:
                    i = pair[0]
                    if poses_data is not None:
                        m_rel_pos = poses_data[i]
                        joint_poses[i] = m_rel_pos / reduction_ratios[i] * joint_signs[i]
                    else:
                        joint_poses[i] = rad2turn(joints_data[i])
                        m_rel_pos = joint_poses[i] * reduction_ratios[i] * joint_signs[i]
                    m_pos = m_rel_pos + motor_home_poses[i]

                    m_pos_min, m_pos_max = motor_ranges[i]
                    motor_range_limit_reached[i] = not (m_pos_min < m_pos < m_pos_max)
                    m_pos = m_pos if m_pos > m_pos_min else m_pos_min
                    m_pos = m_pos if m_pos < m_pos_max else m_pos_max
                    
                    cur_rel_motor_poses[i] = m_rel_pos
                    cur_abs_motor_poses[i] = m_pos
                    results = await safe_set_position(motors[i], position=m_pos, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=True)
                    cur_abs_motor_poses_from_motors[i] = results.values[moteus.Register.POSITION]
                    if moteus.Register.TEMPERATURE in results.values:
                        motor_temperatures[i] = results.values[moteus.Register.TEMPERATURE]
                    else:
                        motor_temperatures[i] = -1.0
                    continue
                i_0, i_1 = pair
                
                if poses_data is not None:
                    m_rel_pos_0 = poses_data[i_0]
                    m_rel_pos_1 = poses_data[i_1]
                    joint_poses[i_0] = (m_rel_pos_0 - m_rel_pos_1) / 2 / reduction_ratios[i_0] * joint_signs[i_0]  # j_pos_shft
                    joint_poses[i_1] = (m_rel_pos_0 + m_rel_pos_1) / 2 / reduction_ratios[i_1] * joint_signs[i_1]  # j_pos_rot
                else:
                    joint_poses[i_0] = rad2turn(joints_data[i_0])
                    joint_poses[i_1] = rad2turn(joints_data[i_1])
                    m_rel_pos_0 = (joint_poses[i_1] * joint_signs[i_1] + joint_poses[i_0] * joint_signs[i_0]) * reduction_ratios[i_0]  # assumed reduction_ratios[i_0] == reduction_ratios[i_1]!
                    m_rel_pos_1 = (joint_poses[i_1] * joint_signs[i_1] - joint_poses[i_0] * joint_signs[i_0]) * reduction_ratios[i_1]  # assumed reduction_ratios[i_0] == reduction_ratios[i_1]!

                # m_pos_0 = data[i_0]
                m_pos_0 = m_rel_pos_0 + motor_home_poses[i_0]
                m_pos_min, m_pos_max = motor_ranges[i_0]
                motor_range_limit_reached[i_0] = not (m_pos_min < m_pos_0 < m_pos_max)
                m_pos_0 = m_pos_0 if m_pos_0 > m_pos_min else m_pos_min
                m_pos_0 = m_pos_0 if m_pos_0 < m_pos_max else m_pos_max
                
                cur_rel_motor_poses[i_0] = m_rel_pos_0
                cur_abs_motor_poses[i_0] = m_pos_0
                results = await safe_set_position(motors[i_0], position=m_pos_0, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=True)
                cur_abs_motor_poses_from_motors[i_0] = results.values[moteus.Register.POSITION]
                if moteus.Register.TEMPERATURE in results.values:
                    motor_temperatures[i_0] = results.values[moteus.Register.TEMPERATURE]
                else:
                    motor_temperatures[i_0] = -1.0

                # m_pos_1 = data[i_1]
                m_pos_1 = m_rel_pos_1 + motor_home_poses[i_1]
                m_pos_min, m_pos_max = motor_ranges[i_1]
                motor_range_limit_reached[i_1] = not (m_pos_min < m_pos_1 < m_pos_max)
                m_pos_1 = m_pos_1 if m_pos_1 > m_pos_min else m_pos_min
                m_pos_1 = m_pos_1 if m_pos_1 < m_pos_max else m_pos_max
                
                cur_rel_motor_poses[i_1] = m_rel_pos_1
                cur_abs_motor_poses[i_1] = m_pos_1
                results = await safe_set_position(motors[i_1], position=m_pos_1, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=True)
                cur_abs_motor_poses_from_motors[i_1] = results.values[moteus.Register.POSITION]
                if moteus.Register.TEMPERATURE in results.values:
                    motor_temperatures[i_1] = results.values[moteus.Register.TEMPERATURE]
                else:
                    motor_temperatures[i_1] = -1.0
            print_status()

        except Exception as e:
            print(f"Error in handler: {e}")
            raise Exception(e)
        finally:
            await asyncio.sleep(handler_period)

async def keep_alive():
    """Periodically send motor positions so the motors don't time out."""
    global results, velocity_limit, motors, active_motor_pair_ids, cur_rel_motor_poses, cur_abs_motor_poses, cur_abs_motor_poses_from_motors, motor_temperatures, joint_poses, reduction_ratios

    while True:
        try:
            for pair in active_motor_pair_ids:
                if len(pair) == 1:
                    i = pair[0]
                    m_pos = cur_abs_motor_poses[i]
                    await safe_set_position(motors[i], position=m_pos, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=False)
                    continue
                i_0, i_1 = pair
                m_pos_0 = cur_abs_motor_poses[i_0]
                await safe_set_position(motors[i_0], position=m_pos_0, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=False)

                m_pos_1 = cur_abs_motor_poses[i_1]
                await safe_set_position(motors[i_1], position=m_pos_1, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=False)
        except Exception as e:
            print("Error sending keep-alive command:", e)
        finally:
            await asyncio.sleep(keep_alive_period)


async def main():
    global results, velocity_limit, motors, active_motor_pair_ids, cur_rel_motor_poses, cur_abs_motor_poses, cur_abs_motor_poses_from_motors, motor_temperatures, joint_poses, reduction_ratios
    
    for pair in active_motor_pair_ids:
        if len(pair) == 1:
            i = pair[0]
            m_pos = cur_abs_motor_poses[i]
            results = await safe_set_position(motors[i], position=m_pos, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=True)
            cur_abs_motor_poses_from_motors[i] = results.values[moteus.Register.POSITION]
            if moteus.Register.TEMPERATURE in results.values:
                motor_temperatures[i] = results.values[moteus.Register.TEMPERATURE]
            else:
                motor_temperatures[i] = -1.0
            joint_poses[i] = m_pos / reduction_ratios[i] * joint_signs[i]
            continue
        i_0, i_1 = pair
        m_pos_0 = cur_abs_motor_poses[i_0]
        results = await safe_set_position(motors[i_0], position=m_pos_0, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=True)
        cur_abs_motor_poses_from_motors[i_0] = results.values[moteus.Register.POSITION]
        if moteus.Register.TEMPERATURE in results.values:
            motor_temperatures[i_0] = results.values[moteus.Register.TEMPERATURE]
        else:
            motor_temperatures[i_0] = -1.0

        m_pos_1 = cur_abs_motor_poses[i_1]
        results = await safe_set_position(motors[i_1], position=m_pos_1, velocity= 0.0, velocity_limit=velocity_limit, accel_limit=10.0, query=True)
        cur_abs_motor_poses_from_motors[i_1] = results.values[moteus.Register.POSITION]
        if moteus.Register.TEMPERATURE in results.values:
            motor_temperatures[i_1] = results.values[moteus.Register.TEMPERATURE]
        else:
            motor_temperatures[i_1] = -1.0
        
        joint_poses[i_0] = (m_pos_0 - m_pos_1) / 2 / reduction_ratios[i_0] * joint_signs[i_0]  # j_pos_shft
        joint_poses[i_1] = (m_pos_0 + m_pos_1) / 2 / reduction_ratios[i_1] * joint_signs[i_1]  # j_pos_rot
    print_status()

    asyncio.create_task(keep_alive())
    asyncio.create_task(handler())

    # Keep the event loop alive
    await asyncio.Future()

    # await motors_set_stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        asyncio.run(motors_set_stop())
        print("\nReceiver stopped.")
    except:
        asyncio.run(motors_set_stop())
