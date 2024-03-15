# Import required libraries
import rospy
import rospkg
import math
import os
import numpy as np
from PIL import Image
import csv

# Initialize the rospkg package
rospack = rospkg.RosPack()

# Dataset info
dataset_name = "seq_hotel"
dataset_speed = 25 # 25 frames per second

# Define variables for the PGM file
resolution = 0.050000
origin = [-100.000000, -100.000000, 0.000000]
width = 4000
height = 4000


# Information for scene creation
init_frames = [0, 1000, 2000, 2700, 4000, 5400, 6800, 7000, 9500, 10200, 11400, 12400, 12850, 14400, 15150]
end_frames = [300, 1300, 2300, 3000, 4300, 5800, 7000, 7400, 9700, 10830, 11600, 12700, 13000, 14600, 15400]
hidden_agent_ids = [8, 48, 68, 89, 101, 116, 130, 143, 175, 222, 265, 284, 306, 344, 363]
sim_start_time = 140.0
sim_speed_limits = [0.3,0.4,0.5,0.6,0.7,1.0,1.8,2.0]

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('dataset_transform_hotel', anonymous=True)

    # Get the path of the desired package
    package_path = rospack.get_path('social_robot_testbed')
    

    # Create a matrix of width x height with values of 0
    matriz = [[255] * width for _ in range(height)]

    # Read the matrix from the file
    homography_matrix_file_path = package_path+'/dataset/'+dataset_name+'/H.txt'
    with open(homography_matrix_file_path, 'r') as file:
        lines = file.readlines()

    # Initialize an empty list to store the matrix values
    homography_matrix = []

    # Parse each line of the file and extract matrix values
    for line in lines:
        values = line.split()
        matrix_row = [float(value) for value in values]
        homography_matrix.append(matrix_row)

    # Convert the list of lists to a NumPy array
    homography_matrix = np.array(homography_matrix)


    # Load the map image
    map_image = Image.open(package_path+'/dataset/'+dataset_name+'/map.png').convert('L')  # Open the image in grayscale

    # Convert the image to a NumPy array
    image_array = np.array(map_image)

    # Find the indices of pixels corresponding to obstacles (white)
    obstacles_indices = np.argwhere(image_array == 255)

    # Iterate over the indices of the obstacles and calculate the positions in meters
    obstacle_positions_homography = []
    for index in obstacles_indices:
        # Get the pixel coordinates
        x, y = index

        # Apply the homography matrix to get the position in meters
        point_in_image = np.array([[x], [y], [1]])
        point_in_meters = np.dot(homography_matrix, point_in_image)
        # point_in_meters[0]+= 5
        # point_in_meters[1]+= 2
        x_pixels = int((origin[0] - point_in_meters[1]) / resolution)
        y_pixels = int((origin[1] + point_in_meters[0]) / resolution)
        matriz[x_pixels][y_pixels] = 0
        obstacle_positions_homography.append([point_in_meters[0], point_in_meters[1]])


    # Create a pgm file
    file_path = os.path.join(package_path, 'maps', dataset_name,  'map.pgm')

    # Create the directory if it does not exist
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(file_path, 'w') as f:
        f.write("P2\n")
        f.write(f"{width} {height}\n")
        f.write("255\n")

        # Write the matrix values
        for row in matriz:
            for value in row:
                f.write(f"{value} ")
            f.write("\n")
    print("Map PGM created successfully.")

    # Create a yaml file
    file_path = os.path.join(package_path, 'maps', dataset_name,  'map.yaml')

    # Create the directory if it does not exist
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(file_path, 'w') as f:
        f.write("image: map.pgm\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
    print("YAML file created successfully.")

    # Create a world file
    file_path = os.path.join(package_path, 'worlds', dataset_name, 'map.world')
    obstacles_world_text = ''

    # Create the directory if it does not exist
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(file_path, 'w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<sdf version="1.6">\n')
        f.write('  <world name="default">\n')

        # Group obstacles based on proximity
        obstacle_groups = {}
        for idx, obstacle_pos in enumerate(obstacle_positions_homography):
            x = obstacle_pos[0][0]
            y = obstacle_pos[1][0]
            rounded_x = round(x, 1)
            rounded_y = round(y, 1)
            if (rounded_x, rounded_y) not in obstacle_groups:
                obstacle_groups[(rounded_x, rounded_y)] = []
            obstacle_groups[(rounded_x, rounded_y)].append((x, y))

        # Write grouped obstacles to the world file
        for group_id, obstacle_group in enumerate(obstacle_groups.values()):
            group_name = f"group_{group_id}"
            group_center_x = sum(pos[0] for pos in obstacle_group) / len(obstacle_group)
            group_center_y = sum(pos[1] for pos in obstacle_group) / len(obstacle_group)
            obstacles_world_text += f'    <model name="{group_name}">\n'
            obstacles_world_text += '      <pose>%.2f %.2f 0.5 0 0 0</pose>\n' % (group_center_x, group_center_y)
            obstacles_world_text += '      <static>true</static>\n'
            obstacles_world_text += '      <link name="link">\n'
            obstacles_world_text += '        <collision name="collision">\n'
            obstacles_world_text += '          <geometry>\n'
            obstacles_world_text += '            <box>\n'
            max_distance = max(np.linalg.norm(np.array(pos) - np.array([group_center_x, group_center_y])) for pos in obstacle_group)
            obstacles_world_text += f'              <size>{max_distance * 2} {max_distance * 2} 1</size>\n'
            obstacles_world_text += '            </box>\n'
            obstacles_world_text += '          </geometry>\n'
            obstacles_world_text += '        </collision>\n'
            obstacles_world_text += '        <visual name="visual">\n'
            obstacles_world_text += '          <geometry>\n'
            obstacles_world_text += '            <box>\n'
            obstacles_world_text += f'              <size>{max_distance * 2} {max_distance * 2} 1</size>\n'
            obstacles_world_text += '            </box>\n'
            obstacles_world_text += '          </geometry>\n'
            obstacles_world_text += '          <material>\n'
            obstacles_world_text += '            <ambient>0.5 0.5 0.5 1</ambient>\n'  # Gray color for obstacles
            obstacles_world_text += '          </material>\n'
            obstacles_world_text += '        </visual>\n'
            obstacles_world_text += '      </link>\n'
            obstacles_world_text += '    </model>\n'

        # Add the ground plane
        obstacles_world_text += '<model name="ground_plane"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><ambient>1 1 1 1</ambient></material></visual></link></model>'
        
        # Add the light source
        obstacles_world_text += '    <light name="sun" type="directional">\n'
        obstacles_world_text += '      <cast_shadows>true</cast_shadows>\n'
        obstacles_world_text += '      <pose>0 0 10 0 0 0</pose>\n'  # Adjust the pose of the light source as needed
        obstacles_world_text += '      <diffuse>0.8 0.8 0.8 1</diffuse>\n'  # Adjust the diffuse color of the light
        obstacles_world_text += '      <specular>0.2 0.2 0.2 1</specular>\n'  # Adjust the specular color of the light
        obstacles_world_text += '      <attenuation>\n'
        obstacles_world_text += '        <range>30</range>\n'  # Adjust the range of the light
        obstacles_world_text += '        <constant>0.9</constant>\n'
        obstacles_world_text += '        <linear>0.01</linear>\n'
        obstacles_world_text += '        <quadratic>0.001</quadratic>\n'
        obstacles_world_text += '      </attenuation>\n'
        obstacles_world_text += '      <direction>-0.5 0.5 -1</direction>\n'  # Adjust the direction of the light
        obstacles_world_text += '    </light>\n'



        f.write(obstacles_world_text)
        f.write('  </world>\n')
        f.write('</sdf>\n')
    

    agents_by_frame = {}
    agents_info = {}


    # Define the file path where the agent information is stored
    file_path = package_path+'/dataset/'+dataset_name+'/obsmat.txt'

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

    # Convert lists to NumPy arrays for easier manipulation (optional)
    for frame_number in agents_by_frame:
        agents_by_frame[frame_number] = np.array(agents_by_frame[frame_number])

    for pedestrian_id in agents_info:
        agents_info[pedestrian_id] = np.array(agents_info[pedestrian_id])

    for scene_index, (init_frame, end_frame) in enumerate(zip(init_frames, end_frames)):
        agents_in_scene = []
        min_frame_number = float('inf')  # Initialize to positive infinity
        max_speed_in_scene = float('-inf')  # Initialize to negative infinity
        # Iterate through the range of frames
        for frame_number in range(init_frame, end_frame + 1):
            if frame_number in agents_by_frame:
                # Iterate through agents in the current frame
                for agent_info in agents_by_frame[frame_number]:
                    agent_frame_number, pedestrian_id, _, _, _, _, _, _ = agent_info
                    # Check if the pedestrian ID is not in the hidden_agent_ids list
                    if pedestrian_id not in agents_in_scene:
                        # Append pedestrian ID to the agents in the scene
                        agents_in_scene.append(pedestrian_id)

        for agent_ids in agents_in_scene:
            prev_agent_info = None
            for agent_info in agents_info[agent_ids]:
                agent_frame_number, pedestrian_id, pos_x1, pos_z1, pos_y1, vel_x1, vel_z1, vel_y1 = agent_info
                
                if min_frame_number > agent_frame_number:
                    min_frame_number = agent_frame_number

                if prev_agent_info is not None:
                    (agent_frame_number2, pedestrian_id2, pos_x2, pos_z2, pos_y2, vel_x2, vel_z2, vel_y2) = prev_agent_info
                    delta_distance = math.sqrt((pos_x1 - pos_x2)**2 + (pos_y1 - pos_y2)**2)
                    delta_time = (agent_frame_number - agent_frame_number2) / dataset_speed
                    
                    if delta_time != 0:  # Avoid division by zero
                        delta_speed = delta_distance / delta_time
                        max_speed_in_scene = max(max_speed_in_scene, delta_speed)
                
                prev_agent_info = agent_info  # Update previous agent's information
    
        for i in range(2): # 0 all agents and 1 for hidden agent
            for speed in sim_speed_limits:
                # Create a world file
                file_path = os.path.join(package_path, 'worlds', dataset_name,'scene'+str(scene_index)+'_'+ str(speed) + '_'+ str(i) +'.world')
                
                speedFactor = max_speed_in_scene/speed

                # Create the directory if it does not exist
                directory = os.path.dirname(file_path)
                if not os.path.exists(directory):
                    os.makedirs(directory)

                with open(file_path, 'w') as f:
                    f.write('<?xml version="1.0" ?>\n')
                    f.write('<sdf version="1.5">\n')
                    f.write('  <world name="default">\n')

                    for agent_ids in agents_in_scene:
                        prev_agent_info = None
                        
                        last_x = None
                        last_y = None
                        last_angle = None
                        last_waypoint_time = None
                        
                        for index_agent_positions, agent_info in enumerate(agents_info[agent_ids]):
                            agent_frame_number, pedestrian_id, pos_x1, pos_z1, pos_y1, vel_x1, vel_z1, vel_y1 = agent_info
                            angle = math.atan2(vel_y1, vel_x1)
                            if index_agent_positions == 0:
                                if i == 0 and pedestrian_id in hidden_agent_ids:
                                    f.write(f'    <!--actor name="actor{int(pedestrian_id)}"><skin><filename>walk.dae</filename></skin><animation name="walking"><filename>walk.dae</filename><interpolate_x>true</interpolate_x></animation><script><trajectory id="0" type="walking">\n')
                                else :
                                    f.write(f'    <actor name="actor{int(pedestrian_id)}"><skin><filename>walk.dae</filename></skin><animation name="walking"><filename>walk.dae</filename><interpolate_x>true</interpolate_x></animation><script><trajectory id="0" type="walking">\n')
                                
                                f.write(f'<waypoint><time>0</time><pose>{pos_x1} {pos_y1} -2 0 0 {angle}</pose></waypoint>\n')
                                
                                waypoint_time = (((agent_frame_number-min_frame_number)/dataset_speed)*speedFactor)+sim_start_time-0.0001
                                f.write(f'<waypoint><time>{waypoint_time}</time><pose>{pos_x1} {pos_y1} -2 0 0 {angle}</pose></waypoint>\n')

                            last_waypoint_time = (((agent_frame_number-min_frame_number)/dataset_speed)*speedFactor)+sim_start_time
                            last_x = pos_x1
                            last_y = pos_y1
                            last_angle = angle
                            f.write(f'<waypoint><time>{last_waypoint_time}</time><pose>{last_x} {last_y} 0 0 0 {last_angle}</pose></waypoint>\n')

                        f.write(f'<waypoint><time>{last_waypoint_time+0.001}</time><pose>{last_x} {last_y} -2 0 0 {last_angle}</pose></waypoint>\n')

                        f.write(f'<waypoint><time>{500}</time><pose>{last_x} {last_y} -2 0 0 {last_angle}</pose></waypoint>\n')
                                
                        if i == 0 and pedestrian_id in hidden_agent_ids:
                            f.write(f'    </trajectory></script></actor-->\n')
                        else :
                            f.write(f'    </trajectory></script></actor>\n')
                        
                    f.write(obstacles_world_text)
                    f.write('  <state world_name="default"><sim_time>130 0</sim_time></state><sim_time>130 0</sim_time></world>\n')
                    f.write('</sdf>\n')
    
    print("World files created successfully.")