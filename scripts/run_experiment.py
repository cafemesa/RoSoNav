import os
import signal
import subprocess
import rospkg
import rospy
import time
import math
from datetime import datetime
import re
import tf
import csv
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Quaternion

# Modify the following variables

navigation_methods = ["unaware", "sf", "rvo" , "sacadrl"] # navigation_methods = ["unaware", "sf", "rvo" , "sacadrl"]
datasets = ["seq_eth", "seq_hotel"] # datasets = ["seq_eth", "seq_hotel"]
robots = ["turtlebot2"] # robots = ["turtlebot2", "evarobot"]
scene_ids = [[0, 1, 2, 3, 4, 5, 6, 7, 8, 9], [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]] # scene_ids = [[0, 1, 2, 3, 4, 5, 6, 7, 8, 9], [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]]
goal_array= [[[12,5],[12,5],[-3,7],[12,5],[0,5],[-1,5],[-3,5],[-3,5],[-3,8],[-3,8]], [[2,-10],[-2,-9],[2.5,-8],[2.5,-8],[2,-10],[2,5],[3,-5],[0,-10],[0,-10],[3,-10],[2,5],[2,4],[4,-8],[3,4],[3,-10]]]
start_array = [[[0,5],[0,5],[7,5],[0,7],[12,5],[9,5],[9,5],[9,5],[9,5],[7,5]],[[2,10],[-2,4],[2.5,3],[2.5,3],[1,0],[2,-5],[3,3],[2,0],[2,3],[2,5],[2,-5],[1,-6],[2,2],[1,-7],[2,2]]]
speeds_values = [0.3,0.4] # speeds_values = [0.3, 0.4, 0.5, 0.7, 1.0, 1.5, 1.8]
methods = ["negotiate", "no_negotiate"] # methods = ["negotiate", "no_negotiate"]

agents_ids = [[7, 48, 77, 124, 150, 159, 189, 226, 273, 303], [8, 48, 68, 89, 101, 116, 130, 143, 175, 222, 265, 284, 306, 344, 363]] # agents_ids = [[7, 48, 77, 124, 150, 159, 189, 226, 273, 303], [8, 48, 68, 89, 101, 116, 130, 143, 175, 222, 265, 284, 306, 344, 363]] for negotiate methods. the users with this IDs are replaced by the robot
intimate_area_radius = 0.45 # To store Time in intimate area
personal_area_radius = 1.2 # To store Time in personal area
distance_collition = 0.45 # Distance to another person to count as collision

# Vaiable for metrics
pedestrian_collisions = 0
environment_collisions = 0
timeouts = 0
success_rate = 0.0
path_lenght = 0.0
time2goal = 0.0
minimum_distance_person = 0.0
average_sṕeed=0.0
minimum_distance_exceeded = 0
id_agents_collitions = []
id_object_collitions = []
robot_positions = []
robot_positions_stamp = []
path_irregularity = []
last_time = 0.0
kill_prcess_time = 20

phase1 = False
phase2 = True



current_agent_pose = None
minimum_distance = float('inf')


# Initialize the rospkg package
rospack = rospkg.RosPack()

# Get the path of the desired package
package_path = rospack.get_path('social_robot_testbed')

def distance_between_points(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def model_states_callback(msg):
    global model_names, model_poses, current_robot_pose
    model_names = msg.name
    model_poses = msg.pose
    for xy in range(len(msg.name)):
        if msg.name[xy] == "mobile_base":
            current_robot_pose = msg.pose[xy]

def match_group(array1, array2):
    for element1 in array1:
        for element2 in array2:
            if element1==element2:
                return True 
    return False

def clock_callback(msg):
    global simulation_process, groups_info, agents_group_info, current_robot_pose, current_clock, current_agent_pose, csv_writer, phase, minimum_distance,last_time, environment_collisions, pedestrian_collisions, timeouts, robot_positions, robot_positions_stamp, id_agents_collitions, minimum_distance_exceeded, path_irregularity
    current_clock = msg.clock.to_sec()

    if msg.clock.to_sec() > 141 and simulation_process:
        all_below_zero = all(pose.position.z < 0 for name, pose in zip(model_names, model_poses) if 'actor' in name.lower())
        
        if all_below_zero and phase == "store_data":
            current_agent_pose = None
            minimum_distance = math.inf
            time.sleep(kill_prcess_time)
            os.killpg(os.getpgid(simulation_process.pid), signal.SIGTERM)
            simulation_process = None
        
        if phase != "store_data":
            delta_distance_goal = distance_between_points(goal_array[x][y][0], goal_array[x][y][1], current_robot_pose.position.x, current_robot_pose.position.y)
            if delta_distance_goal<0.15:
                store_metrics()
                time.sleep(kill_prcess_time)
                os.killpg(os.getpgid(simulation_process.pid), signal.SIGTERM)
                simulation_process = None
                print("Goal")


            if msg.clock.to_sec() > 300 :
                timeouts +=1
                store_metrics()
                time.sleep(kill_prcess_time)
                os.killpg(os.getpgid(simulation_process.pid), signal.SIGTERM)
                simulation_process = None
                print("Timeout")

    if msg.clock.to_sec() > 140 and simulation_process:
        if phase == "store_data":
            
            for i in range(len(model_poses)-1):
                for j in range(i + 1, len(model_poses)):
                    name_i = model_names[i]
                    name_j = model_names[j] 
                    if 'actor' in name_i.lower() and 'actor' in name_j.lower() and model_poses[i].position.z > 0.5 and model_poses[j].position.z > 0.5:
                        dist = distance_between_points(model_poses[i].position.x, model_poses[i].position.y,
                                                       model_poses[j].position.x, model_poses[j].position.y)
                        actor_id_1 = int(re.search(r'\d+$',name_i).group())
                        actor_id_2 = int(re.search(r'\d+$',name_j).group())
                        print("--------")
                        print(actor_id_1)
                        print(actor_id_2)
                        print("--------")
                        if (actor_id_1 in agents_group_info and actor_id_2 in agents_group_info and not match_group(agents_group_info[actor_id_1], agents_group_info[actor_id_2])) or actor_id_1 not in agents_group_info or actor_id_2 not in agents_group_info:
                            minimum_distance = min(minimum_distance, dist)
        else:
            

            if msg.clock.to_sec() > last_time +0.1:
                for i in range(len(model_names)):
                    if 'actor' in model_names[i] and model_names[i] not in id_agents_collitions:
                        dist = distance_between_points(model_poses[i].position.x, model_poses[i].position.y,
                                                        current_robot_pose.position.x, current_robot_pose.position.y)
                        if dist <0.45:
                            id_agents_collitions.append(model_names[i])
                            pedestrian_collisions += 1
                        if minimum_distance>dist:
                            minimum_distance = dist
                if  environment_collisions == 0:
                    for i in range(len(model_names)) :
                        if 'group' in model_names[i]  :
                            dist = distance_between_points(model_poses[i].position.x, model_poses[i].position.y,
                                                            current_robot_pose.position.x, current_robot_pose.position.y)
                            if dist <0.15:
                                environment_collisions += 1

                angle2goal =math.atan2(goal_array[x][y][1] - current_robot_pose.position.y,goal_array[x][y][0] - current_robot_pose.position.x)
                quaternion = (current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w)

                current_robot_angle_rad = tf.transformations.euler_from_quaternion(quaternion)[2]
                diff = angle2goal - current_robot_angle_rad
                diff = diff % (2 * math.pi)
                if diff < 0:
                    diff += 2 * math.pi

                path_irregularity.append(diff)
                robot_positions.append(current_robot_pose)
                robot_positions_stamp.append(msg.clock.to_sec())
                last_time = msg.clock.to_sec()
                    


def store_metrics():
    global simulation_process, current_robot_pose, current_clock, current_agent_pose, csv_writer, phase, minimum_distance,last_time, environment_collisions, pedestrian_collisions, timeouts, robot_positions, robot_positions_stamp, id_agents_collitions, minimum_distance_exceeded, path_irregularity,minimum_distance_person

    path_lenght = 0.0
    time2goal = 0.0

    # Create a pgm file
    file_path_csv_robot_positions = os.path.join(package_path, 'annotations', 'robot_position_'+datasets[x]+'_'+str(scene_ids[x][y])+'_'+robots[z]+'_'+str(speeds_values[u])+'_'+navigation_methods[v]+'.csv')

    # Create the directory if it does not exist
    directory = os.path.dirname(file_path_csv_robot_positions)
    if not os.path.exists(directory):
        os.makedirs(directory)
        
    with open(file_path_csv_robot_positions, 'w', newline='') as file:
        writer = csv.writer(file)

        # Write headers if needed
        writer.writerow(['X Position', 'Y Position'])

        for i in range(len(robot_positions)):
            writer.writerow([robot_positions[i].position.x, robot_positions[i].position.y]) 
            if i>0:
                distance_tmp = distance_between_points(robot_positions[i].position.x, robot_positions[i].position.y, robot_positions[i-1].position.x, robot_positions[i-1].position.y)
                path_lenght +=distance_tmp

    file_path_csv_success_rate = os.path.join(package_path, 'annotations', 'success_rate.csv')

    if not os.path.exists(file_path_csv_success_rate):
        # Create the CSV file with headers
        with open(file_path_csv_success_rate, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Dataset', 'Scene Id ', 'Nav. algorithm',  'Robot', 'Pedestrian Collisions', 'Environment Collisions', "Timeouts" , "Min. Distance Exceeded", "Path Lenght", "Time 2 Goal", "Minimum distance to person", "Avg. Speed", "Path Irregularity"])  # Adjust headers as per your requirement
    
    time2goal = robot_positions_stamp[len(robot_positions_stamp)-1] -  robot_positions_stamp[0]
    average_path_irregularity = sum(path_irregularity) / len(path_irregularity)
    with open(file_path_csv_success_rate, 'a', newline='') as file:
        writer = csv.writer(file)
        # Write your data to the CSV file
        writer.writerow([datasets[x], scene_ids[x][y], navigation_methods[v], robots[z], pedestrian_collisions, environment_collisions, timeouts , 0, path_lenght, time2goal, minimum_distance, path_lenght/time2goal, average_path_irregularity ])
        print("Data appended to CSV file.")

    #reset Metrics numbers
    pedestrian_collisions = 0
    environment_collisions = 0
    timeouts = 0
    success_rate = 0.0
    path_lenght = 0.0
    time2goal = 0.0
    minimum_distance_person = 0.0
    average_sṕeed=0.0
    minimum_distance_exceeded = 0
    id_agents_collitions = []
    id_object_collitions = []
    robot_positions = []
    robot_positions_stamp = []
    path_irregularity = []
    last_time = 0.0
    minimum_distance = float('inf')


if __name__ == "__main__":

    rospy.init_node('social_robot_testbed', anonymous=True)

    rospy.Subscriber("/gazebo/model_states",
                     ModelStates, model_states_callback)
    rospy.Subscriber("/clock", Clock, clock_callback)

    global simulation_process, csv_writer, phase, groups_info, agents_group_info

    agents_group_info = {}
    groups_info = {}
    

    #######################################################################################################
    # Run experiments for default scenarios to get values of the Minimun Distance Esceed Metric
    # Also get the origin and goal position of the selectect agent to be replaced in the negotiate  method - Future work
    #######################################################################################################
    if phase1:

        phase = "store_data"

        # Create a pgm file
        file_path_csv = os.path.join(package_path, 'annotations', 'minimum_distance.csv')

        # Create the directory if it does not exist
        directory = os.path.dirname(file_path_csv)
        if not os.path.exists(directory):
            os.makedirs(directory)

        with open(file_path_csv, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Dataset', 'Scene ID', 'Selected User Distance', 'Origin X', 'Origin Y', 'End X', 'End Y', 'Time'])

            for x in range(len(datasets)):

                ###################################################################
                ###################### get group information ######################
                ###################################################################

                #Define the file path where the group datasetinformation is stored
                
                file_path_groups = package_path+'/dataset/'+datasets[x]+'/groups.txt'

                # Read the agent information from the file
                with open(file_path_groups, 'r') as file:
                    lines = file.readlines()

                # Parse each line of the file and extract information
                for index_groups, line in enumerate(lines):
                    values = line.split()

                    # Check if the index group already exists in the dictionary
                    if index_groups not in groups_info:
                        groups_info[index_groups] = []               

                    for value in values:
                        groups_info[int(index_groups)].append(value)

                        if int(value) not in agents_group_info:
                            agents_group_info[int(value)] = []
                        agents_group_info[int(value)].append(index_groups)


                ###################################################################
                ##################### Get agents information ######################
                ###################################################################

                agents_by_frame = {}
                agents_info = {}


                # Define the file path where the agent information is stored
                file_path = package_path+'/dataset/'+datasets[x]+'/obsmat.txt'

                # Read the agent information from the file
                with open(file_path, 'r') as file:
                    lines = file.readlines()

                # Parse each line of the file and extract information
                for line in lines:
                    values = line.split()
                    # Extract values for each column
                    frame_number = int(round(float(values[0])))
                    pedestrian_id = int(round(float(values[1])))
                    pos_x, pos_z, pos_y = float(values[2]), float(values[3]), float(values[4])
                    vel_x, vel_z, vel_y = float(values[5]), float(values[6]), float(values[7])
                    agent_info = (frame_number, pedestrian_id, pos_x, pos_z, pos_y, vel_x, vel_z, vel_y)

                    # Check if the frame number already exists in the dictionary
                    if frame_number not in agents_by_frame:
                        agents_by_frame[frame_number] = []
                    
                    # Check if the agent id already exists in the dictionary
                    if pedestrian_id not in agents_info:
                        agents_info[pedestrian_id] = []
                    
                    # Append agent information to the list for the corresponding frame number
                    agents_by_frame[frame_number].append(agent_info)
                    agents_info[pedestrian_id].append(agent_info)

                ###################################################################
                ######################### Run simulations #########################
                ###################################################################

                for y in range(len(scene_ids[x])):
                    roslaunch_cmd = f"roslaunch social_robot_testbed default_scenarios.launch dataset:={datasets[x]} scene_id:={scene_ids[x][y]}"
                    print (roslaunch_cmd)
                    simulation_process = subprocess.Popen(roslaunch_cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                    out, err = simulation_process.communicate()  # Capture the process output
                    

        print("Phase 1: Done")

    #######################################################################################################
    # Run experiments for Scenarios where the robot make all the efford to avoid
    #######################################################################################################

    if phase2:

        phase = "run_simulation"
        
        for x in range(len(datasets)):
            for u in range(len(speeds_values)):
                for y in range(len(scene_ids[x])):
                    for z in range(len(robots)):
                        for v in range(len(navigation_methods)):                         
                            robot_angle = math.atan2(goal_array[x][y][1]-start_array[x][y][1], goal_array[x][y][0]-start_array[x][y][0])
                            roslaunch_cmd = f"roslaunch social_robot_testbed navigation_test.launch dataset:={datasets[x]} scene_id:={scene_ids[x][y]} robot:={robots[z]} speed:={speeds_values[u]} method:={navigation_methods[v]} robot_origin_x:={start_array[x][y][0]} robot_origin_y:={start_array[x][y][1]} robot_origin_yaw:={robot_angle} robot_goal_x:={goal_array[x][y][0]} robot_goal_y:={goal_array[x][y][1]}"
                            print (roslaunch_cmd)
                            simulation_process = subprocess.Popen(roslaunch_cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                            out, err = simulation_process.communicate()  # Capture the process output
                            

        print("Phase 2:Done")            
    
    rospy.spin()
