#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/PointCloud2.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <regex>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Global variables to store data
ros::Publisher cmd_vel_pub_;
bool goal_reached_ = false;
double goal_x, goal_y, linear_velocity, angular_velocity;
geometry_msgs::Pose current_robot_pose;
gazebo_msgs::ModelStates modelState;


// Function declarations
void clockCallback(const rosgraph_msgs::Clock &msg);
void stopRobot();
void rotateRobot(double angle_error);
void moveForward(double distance_to_goal);
void rotateAndMoveRobot(double angle_error, double distance_to_goal);
double normalizeAngle(double angle);
void modelCallback(const gazebo_msgs::ModelStates msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "unaware_robot_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param<double>("goal_x", goal_x, 0.700344);
    nh.param<double>("goal_y", goal_y, 4.908462);
    nh.param<double>("linear_velocity", linear_velocity, 0.3);
    nh.param<double>("angular_velocity", angular_velocity, 0.5);

    // Subscribers
    ros::Subscriber subClock = n.subscribe("/clock", 1000, clockCallback);
    ros::Subscriber subModelStates = n.subscribe("/gazebo/model_states", 1000, modelCallback);

    // Publisher for MoveBase simple goal
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Enter the ROS event loop
    ros::spin();
}

// Callback for the clock topic
void clockCallback(const rosgraph_msgs::Clock &msg)
{
    if (msg.clock.toSec() > 140 && !goal_reached_)
    {
        double distance_to_goal = std::sqrt(std::pow(goal_x - current_robot_pose.position.x, 2) + std::pow(goal_y - current_robot_pose.position.y, 2));

            if (distance_to_goal < 0.1)
            {
                goal_reached_ = true;
                stopRobot();
            }
            else
            {

                // Get robot angle in radians
                tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
                tf2::Matrix3x3 matrix(quaternion);
                double roll_robot, pitch_robot, yaw_robot;
                matrix.getRPY(roll_robot, pitch_robot, yaw_robot);

                double target_angle = std::atan2(goal_y - current_robot_pose.position.y, goal_x - current_robot_pose.position.x);
                double angle_error = normalizeAngle(target_angle - yaw_robot);

                if (std::fabs(angle_error) > 0.5)
                {
                    rotateRobot(angle_error);
                }
                else if (std::fabs(angle_error) > 0.1)
                {
                    rotateAndMoveRobot(angle_error, distance_to_goal);
                }
                else
                {
                    moveForward(distance_to_goal);
                }
            }
    }
}

void stopRobot()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

/**
 * @brief Function that control the rotation of the robot
 * @param angle_error between the current orientation of the robot and the orientation between the robot position and next way point
*/
void rotateRobot(double angle_error)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = angle_error;
    cmd_vel_pub_.publish(cmd_vel);
}

/**
 * @brief Function that control the translation of the robot
 * @param distance_to_goal distance between the current robot postion and the position of the next way point
*/
void moveForward(double distance_to_goal)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

/**
 * @brief Function that control the rotation of the robot
 * @param angle_error between the current orientation of the robot and the orientation between the robot position and next way point
 * @param distance_to_goal distance between the current robot postion and the position of the next way point
*/
void rotateAndMoveRobot(double angle_error, double distance_to_goal)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angle_error*3;
    cmd_vel_pub_.publish(cmd_vel);
}

/**
 * @brief Function that Normalize angle between -pi and pi
 * @param angle to be normlized
*/
double normalizeAngle(double angle)
{
    while (angle <= -M_PI)
    {
        angle += 2 * M_PI;
    }
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    return angle;
}

/**
 * @brief Callback function to get the data from gazebo
 * @param msg Data
*/
void modelCallback(const gazebo_msgs::ModelStates msg)
{
    modelState = msg;
    for (int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == "mobile_base")
        {
            current_robot_pose = msg.pose[i];
        }
    }
}
