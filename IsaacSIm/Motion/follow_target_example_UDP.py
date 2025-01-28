from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from tasks.follow_target import FollowTarget
import numpy as np
import time
import socket
import hashlib
from controllers.rmpflow import RMPFlowController

interval_sec = 0.02
# Configure the UDP client
UDP_IP = "192.168.1.112"  # Jetson's IP address
UDP_PORT = 5005           # Port to send to

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def add_checksum(data):
    """Add checksum to the data."""
    checksum = hashlib.md5(data.encode()).hexdigest()
    return f"{data}|{checksum}"

def parse_response(response):
    """Parse the response from the server."""
    try:
        message, checksum = response.rsplit("|", 1)
        calculated_checksum = hashlib.md5(message.encode()).hexdigest()
        if calculated_checksum == checksum:
            parts = message.split()
            if parts[0] == "CURRENT_STATE":
                positions = list(map(float, parts[1:9]))
                temperatures = list(map(float, parts[9:]))
                return positions, temperatures
        else:
            print("Checksum mismatch in response!")
    except Exception as e:
        print(f"Error parsing response: {e}")
    return None, None

my_world = World(stage_units_in_meters=1.0)
#Initialize the Follow Target task with a target location for the cube to be followed by the end effector
my_task = FollowTarget(name="skyentific_follow_target", target_position=np.array([0.5, 0, 0.5]))
my_world.add_task(my_task)
my_world.reset()
task_params = my_world.get_task("skyentific_follow_target").get_params()
target_name = task_params["target_name"]["value"]
skyentific_name = task_params["robot_name"]["value"]
my_skyentific = my_world.scene.get_object(skyentific_name)
#initialize the controller
my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_skyentific)
my_controller.reset()
articulation_controller = my_skyentific.get_articulation_controller()

last_time = time.time()
joint_positions = [0.0] * 8  # Initial positions
gen = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        articulation_controller.apply_action(actions)
        #send udp
        joint_positions = observations["yellowarm_robot"]["joint_positions"][:8]
        current_time = time.time()
        if current_time - last_time>= interval_sec:
            try:
                # Create the command
                positions_str = " ".join(f"{pos:.3f}" for pos in joint_positions)
                command = f"SET_JOINT_POSITIONS {positions_str}"
                message = add_checksum(command)
                sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
                #print(f"Sent: {command}")
                
                # Receive and parse the response
                sock.settimeout(1)  # 1-second timeout
                response, _ = sock.recvfrom(1024)
                positions, temperatures = parse_response(response.decode())
                last_time += interval_sec
                gen += 1
                if temperatures and gen > 50:
                    formatted_line = " | ".join([f"{temp:.2f}" for i, temp in enumerate(temperatures)])
                    #print(f"\rJoint temperatures: {temperatures}", end="")
                    gen = 0
                    print(f"Joint temperatures: {formatted_line}")
            except socket.timeout:
                print("No response from server, retrying...")
            except Exception as e:
                print(f"Error in control loop: {e}")
        
simulation_app.close()
