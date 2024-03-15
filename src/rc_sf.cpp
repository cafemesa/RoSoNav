#include "ros/ros.h"
#include "include/lightsfm/sfm.hpp"
#include "rosgraph_msgs/Clock.h"
#include "geometry_msgs/Twist.h"
#include "include/rvo/RVO.h"
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
std::string method;
ros::Publisher cmd_vel_pub_;
bool goal_reached_ = false;
double goal_x, goal_y, linear_velocity, angular_velocity;
geometry_msgs::Pose current_robot_pose;
gazebo_msgs::ModelStates modelState;
sensor_msgs::PointCloud2 currentObstaclePointCloud;
double last_processed_time = 0.0;

struct DatasetLine
{
    double frameId, personId, positionX, positionY, positionZ, speedX, speedY, speedZ;

    DatasetLine(double col1, double col2, double col3, double col4, double col5, double col6, double col7, double col8)
        : frameId(col1), personId(col2), positionX(col3), positionY(col5), positionZ(col4), speedX(col6), speedY(col8), speedZ(col7) {}
};

std::vector<DatasetLine> datasetLineVector;
bool annotationProcessed = false;

struct AgentPositionStored
{
    std::string name;
    double x;
    double y;
    double time;

    AgentPositionStored(std::string nameIn, double xIn, double yIn, double timeIn)
        : name(nameIn), x(xIn), y(yIn), time(timeIn) {}
};
std::vector<AgentPositionStored> agentPositionStored;

// Function declarations
void clockCallback(const rosgraph_msgs::Clock &msg);
void stopRobot();
void rotateRobot(double angle_error);
void moveForward(double distance_to_goal);
void getAnnotationData();
void rotateAndMoveRobot(double angle_error, double distance_to_goal);
double normalizeAngle(double angle);
void modelCallback(const gazebo_msgs::ModelStates msg);
void obstaclesPointCloudCallback(const sensor_msgs::PointCloud2 msg);
int getIndexByName(const std::vector<AgentPositionStored> vec, const std::string targetName);

// Motion Controllers
void socialForceMotionController(double currentTime);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_social_force_model");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    if (!annotationProcessed)
    {
        getAnnotationData();
        annotationProcessed = !annotationProcessed;
    }

    // Read parameters from the parameter server
    nh.param<std::string>("method", method, "apa");
    nh.param<double>("goal_x", goal_x, 2);
    nh.param<double>("goal_y", goal_y, -5);
    nh.param<double>("linear_velocity", linear_velocity, 0.45);
    nh.param<double>("angular_velocity", angular_velocity, 0.3);

    // Subscribers
    ros::Subscriber subClock = n.subscribe("/clock", 1000, clockCallback);
    ros::Subscriber subModelStates = n.subscribe("/gazebo/model_states", 1000, modelCallback);
    ros::Subscriber subObstaclesPointCloud = n.subscribe("/pointcloud_traversable", 1000, obstaclesPointCloudCallback);

    // Publisher for MoveBase simple goal
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Enter the ROS event loop
    ros::spin();
}

void stopRobot()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

/**
 * @brief Callback function to get the clock time and excecute the control at 10 hz
 * @param msg Data
 */
void clockCallback(const rosgraph_msgs::Clock &msg)
{
    if (msg.clock.toSec() > 140 && !goal_reached_ && msg.clock.toSec() > last_processed_time + 0.1)
    {
        socialForceMotionController(msg.clock.toSec());
        last_processed_time = msg.clock.toSec();
    }
}

/**
 * @brief Function that control the rotation of the robot
 * @param angle_error between the current orientation of the robot and the orientation between the robot position and next way point
 */
void rotateRobot(double angle_error)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = angle_error/2;
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
    cmd_vel.angular.z = angle_error /2;
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

void socialForceMotionController(double currentTime)
{

    // Get obstacles
    pcl::PointCloud<pcl::PointXYZI> currentObstaclePCL;
    pcl::fromROSMsg(currentObstaclePointCloud, currentObstaclePCL);
    std::vector<utils::Vector2d> laserObstaclesTmp;
    for (int j = 0; j < currentObstaclePCL.size(); j++)
    {
        for (int i = 0; i < modelState.name.size(); i++)
        {
            std::regex pattern("actor(\\d+)");
            std::smatch matches;

            if (modelState.name[i].find("actor") != std::string::npos && std::regex_search(modelState.name[i], matches, pattern) && modelState.pose[i].position.z > 0)
            {
                double distance2obstacle = (double)sqrt(pow(currentObstaclePCL[j].x - modelState.pose[i].position.x, 2) + pow(currentObstaclePCL[j].y - modelState.pose[i].position.y, 2));
                if (distance2obstacle > 0.45 && distance2obstacle < 3.0)
                {
                    laserObstaclesTmp.push_back(utils::Vector2d(currentObstaclePCL[j].x, currentObstaclePCL[j].y));
                }
            }
        }
        
    }

    // Define agent vector
    std::vector<sfm::Agent> myAgents;

    // Extract agents position from model
    for (int i = 0; i < modelState.name.size(); i++)
    {
        std::regex pattern("actor(\\d+)");
        std::smatch matches;

        if (modelState.name[i].find("actor") != std::string::npos && std::regex_search(modelState.name[i], matches, pattern) && modelState.pose[i].position.z > 0)
        {
            // Extract position
            utils::Vector2d agentPosition = utils::Vector2d(modelState.pose[i].position.x, modelState.pose[i].position.y);

            // Extract orientation
            tf2::Quaternion quaternion(modelState.pose[i].orientation.x, modelState.pose[i].orientation.y, modelState.pose[i].orientation.z, modelState.pose[i].orientation.w);
            tf2::Matrix3x3 matrix(quaternion);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            utils::Angle agentYaw = utils::Angle::fromRadian(yaw);

            int objIndex = getIndexByName(agentPositionStored, modelState.name[i]);
            double agentSpeed = 0.0;
            if (objIndex >= 0)
            {
                double deltaDistance = (double)sqrt(pow(modelState.pose[i].position.x - agentPositionStored[objIndex].x, 2) + pow(modelState.pose[i].position.y - agentPositionStored[objIndex].y, 2));
                double deltaTime = currentTime - agentPositionStored[objIndex].time;
                agentSpeed = deltaDistance / deltaTime;
                agentPositionStored[objIndex].x = modelState.pose[i].position.x;
                agentPositionStored[objIndex].y = modelState.pose[i].position.y;
                agentPositionStored[objIndex].time = currentTime;
            }
            else
            {
                agentPositionStored.push_back(AgentPositionStored(modelState.name[i], modelState.pose[i].position.x, modelState.pose[i].position.y, currentTime));
            }

            // Create Agent OBJ
            sfm::Agent myAgent = sfm::Agent(agentPosition, agentYaw, agentSpeed, 0.0);
            myAgent.desiredVelocity = agentSpeed;
            myAgent.radius = 1.2;
            myAgent.obstacles1 = laserObstaclesTmp;

            // Set Agent Goal
            // sfm::Goal goal;
            // goal.radius = 0.3;
            // goal.center.set(0.0, 0.0);
            // myAgent.goals.push_back(goal);

            // myAgent.params.forceFactorObstacle = 5.0;

            myAgents.push_back(myAgent);
        }
    }

    // Create Robot Agent
    // Extract position
    utils::Vector2d sfmRobotPosition = utils::Vector2d(current_robot_pose.position.x, current_robot_pose.position.y);

    // Extract orientation
    tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    utils::Angle sfmRobotYaw = utils::Angle::fromRadian(yaw);

    sfm::Agent myRobotAgent = sfm::Agent(sfmRobotPosition, sfmRobotYaw, linear_velocity, 0.0);
    myRobotAgent.radius = 1.2;

    sfm::Goal robotGoal;
    robotGoal.center.set(goal_x, goal_y);
    myRobotAgent.goals.push_back(robotGoal);
    myAgents.push_back(myRobotAgent);

    myAgents = sfm::SFM.computeForces(myAgents);

    double robotForceAngle = atan2(myAgents.back().forces.globalForce.getY(), myAgents.back().forces.globalForce.getX());
    // double robotForceAngle = myAgents.back().forces.globalForce.angle().toRadian();
    double force = myAgents.back().forces.socialForce.angle().toRadian();

    double distance_to_goal = std::sqrt(std::pow(goal_x - current_robot_pose.position.x, 2) + std::pow(goal_y - current_robot_pose.position.y, 2));

    if (distance_to_goal < 0.1)
    {
        goal_reached_ = true;
        stopRobot();
    }
    else
    {
        ROS_INFO("STEP");

        double angle_error = normalizeAngle(robotForceAngle - yaw);

        if (std::fabs(angle_error) > 0.5)
        {
            rotateRobot(angle_error);
        }
        else if (std::fabs(angle_error) > 0.05)
        {
            rotateAndMoveRobot(angle_error, distance_to_goal);
        }
        else
        {
            moveForward(distance_to_goal);
        }
    }
}

void obstaclesPointCloudCallback(const sensor_msgs::PointCloud2 msg)
{
    currentObstaclePointCloud = msg;
}

int getIndexByName(const std::vector<AgentPositionStored> vec, const std::string targetName)
{
    for (size_t i = 0; i < vec.size(); ++i)
    {
        if (vec[i].name == targetName)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void getAnnotationData()
{
    std::string inputfile = "/home/carlos/catkin_ws/src/social_navigation_testbed/dataset/seq_eth/obsmat.txt";

    std::ifstream infile(inputfile);
    if (infile.fail())
    {
        std::cout << "Failed to open file: " << inputfile << std::endl;
        return;
    }

    // Store all dataset lines in datasetLineVector and store the max speed in the scene
    double col1, col2, col3, col4, col5, col6, col7, col8;
    while (infile >> col1 >> col2 >> col3 >> col4 >> col5 >> col6 >> col7 >> col8)
    {
        // Get dataset line
        DatasetLine datasetLine(col1, col2, col3, col4, col5, col6, col7, col8);
        // Store dataset line in datasetLineVector
        datasetLineVector.push_back(datasetLine);
    }
    infile.close();
}