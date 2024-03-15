import os
import math
import rospy
import gym
import numpy as np
import tf
from geometry_msgs.msg import Quaternion, Twist
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates

from gym import spaces
from gym_collision_avoidance.envs.agent import Agent
from gym_collision_avoidance.envs.policies.GA3CCADRLPolicy import GA3CCADRLPolicy
from gym_collision_avoidance.envs.dynamics.UnicycleDynamics import UnicycleDynamics
from gym_collision_avoidance.envs.sensors.OtherAgentsStatesSensor import OtherAgentsStatesSensor
from gym_collision_avoidance.envs.policies.NonCooperativePolicy import (
    NonCooperativePolicy,
)
from gym_collision_avoidance.envs.policies.CADRLPolicy import CADRLPolicy
from gym_collision_avoidance.envs.sensors.LaserScanSensor import LaserScanSensor

from gym_collision_avoidance.envs.collision_avoidance_env import (
    CollisionAvoidanceEnv,
)
# Set up logger and environment
gym.logger.set_level(40)
os.environ["GYM_CONFIG_CLASS"] = "Example"

# Global variables
agent_positions_current = None
agent_positions_last = None
clock_current = 0.0
clock_last = 0.0
clock_last_2 = 0.0
robot_position_current = None
robot_position_last = None
goal_x = 2.0  # Default value
goal_y = -5.0  # Default value
linear_velocity = 0.45  # Default value
angular_velocity = 0.3  # Default value
target_angle = 0.0
cmd_vel_pub = None
target_position = None
goal_reached = False
clock_pub_last = 0.0


def processData():
    global target_position, clock_current, target_angle, clock_last, env, agent_positions_current, agent_positions_last, robot_position_current, robot_position_last, goal_x, goal_y, linear_velocity, angular_velocity, cmd_vel_pub, goal_reached

    

    # Define Agento Array
    agents_array = []

    if robot_position_current != None and (clock_last == 0.0 or clock_current - clock_last >= 0.5):
        distance_to_goal = math.sqrt((goal_x-robot_position_current.position.x)**2 + (goal_y-robot_position_current.position.y)**2)
        if distance_to_goal>0.1:
            quaternion_robot = [
                robot_position_current.orientation.x,
                robot_position_current.orientation.y,
                robot_position_current.orientation.z,
                robot_position_current.orientation.w
            ]
            roll_robot, pitch_robot, yaw_robot = tf.transformations.euler_from_quaternion(quaternion_robot)

            curren_robot = Agent(robot_position_current.position.x, robot_position_current.position.y, goal_x, goal_y,0.3, linear_velocity, yaw_robot, GA3CCADRLPolicy, UnicycleDynamics, [OtherAgentsStatesSensor], -1)
            agents_array.append(curren_robot)
        
            if agent_positions_current != None and target_position == None:

                for i in range(len(agent_positions_current.name)):
                    if 'actor' in agent_positions_current.name[i] and agent_positions_current.pose[i].position.z > 0:
                        current_speed = 0.0 # DEBUG
                        quaternion = [
                            agent_positions_current.pose[i].orientation.x,
                            agent_positions_current.pose[i].orientation.y,
                            agent_positions_current.pose[i].orientation.z,
                            agent_positions_current.pose[i].orientation.w
                        ]

                        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
                        current_goal_x = agent_positions_current.pose[i].position.x + current_speed*math.cos(yaw)
                        current_goal_y = agent_positions_current.pose[i].position.y + current_speed*math.sin(yaw)

                        if agent_positions_last != None:
                            for j in range(len(agent_positions_last.name)):
                                if agent_positions_current.name[i] == agent_positions_last.name[j]:
                                    delta_y = agent_positions_current.pose[i].position.y-agent_positions_last.pose[j].position.y
                                    delta_x = agent_positions_current.pose[i].position.x-agent_positions_last.pose[j].position.x
                                    yaw = math.atan2(delta_y, delta_x)
                                    delta_distance = math.sqrt((delta_x)**2 + (delta_y)**2)
                                    current_speed = delta_distance / (clock_current - clock_last)
                                    current_goal_x = agent_positions_current.pose[i].position.x + current_speed*math.cos(yaw)
                                    current_goal_y = agent_positions_current.pose[i].position.y + current_speed*math.sin(yaw)

                        curren_agent = Agent(agent_positions_current.pose[i].position.x, agent_positions_current.pose[i].position.y, current_goal_x, current_goal_y, 0.6, current_speed, yaw, NonCooperativePolicy, UnicycleDynamics, [OtherAgentsStatesSensor], i)
                        agents_array.append(curren_agent)

            clock_last = clock_current
            agent_positions_last = agent_positions_current

            if len(agents_array) >1 :

                # Set agent configuration (start/goal pos, radius, size, policy)
                collision_avoidance_env = CollisionAvoidanceEnv()
                agents = agents_array
                [
                    agent.policy.initialize_network()
                    for agent in agents
                    if hasattr(agent.policy, "initialize_network")
                ]
                collision_avoidance_env.set_agents(agents)
                collision_avoidance_env.collision_dist = 0.50
                collision_avoidance_env.map

                obs = collision_avoidance_env.reset()
                actions = {}

                obs, rewards, terminated, truncated, which_agents_done = collision_avoidance_env.step(
                    actions,(-linear_velocity+1.3)
                )
                target_position = [agents[0].pos_global_frame[0], agents[0].pos_global_frame[1]]
                obs = collision_avoidance_env.reset()
            # else:
            #     target_position = None



def model_states_callback(msg):
    global agent_positions_current, robot_position_current
    agent_positions_current = msg
    if 'mobile_base' in msg.name:
        robot_position_current = msg.pose[msg.name.index('mobile_base')]

def clock_callback(msg):
    global clock_current, robot_position_current, target_position, clock_pub_last, goal_reached
    clock_current = msg.clock.to_sec()

    if robot_position_current!= None  and (clock_pub_last == 0.0 or clock_current - clock_pub_last >= 0.05):
        distance_to_goal = math.sqrt((goal_x-robot_position_current.position.x)**2 + (goal_y-robot_position_current.position.y)**2)

        print(distance_to_goal)

        if distance_to_goal>0.15 and goal_reached ==False:
            quaternion_robot = [
                robot_position_current.orientation.x,
                robot_position_current.orientation.y,
                robot_position_current.orientation.z,
                robot_position_current.orientation.w
            ]
            roll_robot, pitch_robot, yaw_robot = tf.transformations.euler_from_quaternion(quaternion_robot)
            delta_y = goal_y - robot_position_current.position.y
            delta_x = goal_x - robot_position_current.position.x
            target_angle = math.atan2(delta_y , delta_x)
            angle_error = normalize_angle(target_angle - yaw_robot)

            if target_position != None:
                distance_to_waypoint = math.sqrt((target_position[0]-robot_position_current.position.x)**2 + (target_position[1]-robot_position_current.position.y)**2)

                distance_to_goal = math.sqrt((goal_x-robot_position_current.position.x)**2 + (goal_y-robot_position_current.position.y)**2)

                if distance_to_waypoint > 0.1:
                    delta_y = target_position[1] - robot_position_current.position.y
                    delta_x = target_position[0] - robot_position_current.position.x
                    target_angle = np.arctan2(delta_y , delta_x)
                    angle_error = normalize_angle(target_angle - yaw_robot)
                    print("waypoint - " , distance_to_waypoint)
                    print("goal - " , distance_to_goal)
                    print("angle_error - " , angle_error)
                else:    
                    target_position = None
            else:
                print("goal - " , distance_to_goal)
                print("angle_error - " , angle_error)

            if clock_current > 140:
                # stop_robot()
                if abs(angle_error) > 0.5:
                    rotate_robot(angle_error)
                elif abs(angle_error) > 0.05:
                    rotate_and_move_robot(angle_error, distance_to_goal)
                else:
                    move_forward(distance_to_goal)
        else:
            goal_reached = True
        clock_pub_last = clock_current


def normalize_angle(angle):
    while angle <= -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle

def rotate_robot(angle_error):
    global cmd_vel_pub
    print(linear_velocity)
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = angle_error/2
    cmd_vel_pub.publish(cmd_vel)

def stop_robot():
    global cmd_vel_pub
    print(linear_velocity)
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)

def move_forward(distance_to_goal):
    global cmd_vel_pub
    print(linear_velocity)
    cmd_vel = Twist()
    cmd_vel.linear.x = linear_velocity
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)

def rotate_and_move_robot(angle_error, distance_to_goal):
    global cmd_vel_pub
    print(linear_velocity)
    cmd_vel = Twist()
    cmd_vel.linear.x = linear_velocity
    cmd_vel.angular.z =  angle_error/2
    cmd_vel_pub.publish(cmd_vel)

# Main function
def main():
    global goal_x, goal_y, linear_velocity, angular_velocity, cmd_vel_pub
    rospy.init_node('collision_avoidance_node')
    
    # Retrieve parameter values from the parameter server
    goal_x = rospy.get_param("~goal_x", goal_x)
    goal_y = rospy.get_param("~goal_y", goal_y)
    linear_velocity = rospy.get_param("~linear_velocity", linear_velocity)
    angular_velocity = rospy.get_param("~angular_velocity", angular_velocity)
    
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    rospy.Subscriber('/clock', Clock, clock_callback)
    
    cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    
    rate = rospy.Rate(1000)  # 10 Hz
    
    while not rospy.is_shutdown():
        processData()
        rate.sleep()


if __name__ == "__main__":
    main()
    print("Experiment over.")
