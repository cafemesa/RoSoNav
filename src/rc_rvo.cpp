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
ros::Publisher cmd_vel_pub_;
bool goal_reached_ = false;
double goal_x, goal_y, linear_velocity, angular_velocity;
geometry_msgs::Pose current_robot_pose;
gazebo_msgs::ModelStates modelState;
sensor_msgs::PointCloud2 currentObstaclePointCloud;
double initial_robot_gazebo_position_x = 12.091963;
double initial_robot_gazebo_position_y = 5.868032;
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

struct ApaAgent
{
    int personId;
    double x;
    double y;
    double a;
    double speed;
    double time;

    ApaAgent(int personIdIn, double xIn, double yIn, double aIn, double timeIn, double speedIn)
        : personId(personIdIn), x(xIn), y(yIn), a(aIn), time(timeIn), speed(speedIn) {}
};

// Function declarations
void clockCallback(const rosgraph_msgs::Clock &msg);
void stopRobot();
void rotateRobot(double angle_error);
void moveForward(double distance_to_goal);
void rotateAndMoveRobot(double angle_error, double distance_to_goal);
double normalizeAngle(double angle);
void modelCallback(const gazebo_msgs::ModelStates msg);

// Motion Controllers
void rvo2MotionController(double currentTime);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "social_navigation_testbed_robot_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Read parameters from the parameter server
    nh.param<double>("goal_x", goal_x, 0.700344);
    nh.param<double>("goal_y", goal_y, 4.908462);
    nh.param<double>("linear_velocity", linear_velocity, 0.5);
    nh.param<double>("angular_velocity", angular_velocity, 0.3);

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
    if (!goal_reached_ && msg.clock.toSec() > 140  && msg.clock.toSec() > last_processed_time + 0.1)
    {
            rvo2MotionController(msg.clock.toSec());
            last_processed_time = msg.clock.toSec();
    }
}

void stopRobot()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

void rotateRobot(double angle_error)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = angle_error;
    cmd_vel_pub_.publish(cmd_vel);
}

void moveForward(double distance_to_goal)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

void rotateAndMoveRobot(double angle_error, double distance_to_goal)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angle_error*3;
    cmd_vel_pub_.publish(cmd_vel);
}

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

// Callback for the model states topic
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

void rvo2MotionController(double currentTime)
{
    // Create a new simulator instance.
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();

    // Specify default parameters for agents that are subsequently added.
    sim->setAgentDefaults(1.2f, 10, 3.0f, 5.0f, 1.2f, linear_velocity);

    // Set up the scenario.
    // Specify global time step of the simulation.
    sim->setTimeStep(0.1f);

    // Specify default parameters for agents that are subsequently added.
    // sim->setAgentDefaults(15.0f, 10, 10.0f, 5.0f, 2.0f, 2.0f); // TODO Check defaults

    // Add agents, specifying their start position.
    for (int i = 0; i < modelState.name.size(); i++)
    {
        std::regex pattern("actor(\\d+)");
        std::smatch matches;

        if (modelState.name[i].find("actor") != std::string::npos && std::regex_search(modelState.name[i], matches, pattern) && modelState.pose[i].position.z > 0)
        {
            sim->addAgent(RVO::Vector2(modelState.pose[i].position.x, modelState.pose[i].position.y));

            for (int j = datasetLineVector.size() - 1; j >= 0; j--)
            {
                if (datasetLineVector[j].personId == stoi(matches[1].str()))
                {
                    sim->setAgentPrefVelocity(sim->getNumAgents() - 1, normalize(RVO::Vector2(datasetLineVector[j].positionX, datasetLineVector[j].positionY) - sim->getAgentPosition(sim->getNumAgents() - 1)));
                    break;
                }
            }
        }
    }

    // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.// TODO Check
    pcl::PointCloud<pcl::PointXYZI> currentObstaclePCL;
    pcl::fromROSMsg(currentObstaclePointCloud, currentObstaclePCL);
    for (int j = 0; j < currentObstaclePCL.size(); j++)
    {
        bool isPerson = false;

        for (int i = 0; i < sim->getNumAgents(); i++)
        {
            double distance2obstacle = (double)sqrt(pow(initial_robot_gazebo_position_x - currentObstaclePCL[j].x - sim->getAgentPosition(i).x(), 2) + pow(initial_robot_gazebo_position_y - currentObstaclePCL[j].y - sim->getAgentPosition(i).y(), 2));
            if (distance2obstacle < 0.4)
            {
                isPerson = true;
                break;
            }
        }
        if (!isPerson)
        {

            std::vector<RVO::Vector2> vertices;
            vertices.push_back(RVO::Vector2(initial_robot_gazebo_position_x - currentObstaclePCL[j].x, initial_robot_gazebo_position_y - currentObstaclePCL[j].y));
            vertices.push_back(RVO::Vector2(initial_robot_gazebo_position_x - currentObstaclePCL[j].x + 0.05, initial_robot_gazebo_position_y - currentObstaclePCL[j].y));
            vertices.push_back(RVO::Vector2(initial_robot_gazebo_position_x - currentObstaclePCL[j].x + 0.05, initial_robot_gazebo_position_y - currentObstaclePCL[j].y + 0.05));
            vertices.push_back(RVO::Vector2(initial_robot_gazebo_position_x - currentObstaclePCL[j].x, initial_robot_gazebo_position_y - currentObstaclePCL[j].y + 0.05));
            sim->addObstacle(vertices);
        }
    }
    sim->processObstacles();

    // Add robot position and goal
    sim->addAgent(RVO::Vector2(current_robot_pose.position.x, current_robot_pose.position.y));
    sim->setAgentPrefVelocity(sim->getNumAgents() - 1, normalize(RVO::Vector2(goal_x, goal_y) - sim->getAgentPosition(sim->getNumAgents() - 1)));

    // Perform (and manipulate) the simulation.
    sim->doStep();

    // Extract Current Robot orientation
    tf2::Quaternion quaternion(current_robot_pose.orientation.x, current_robot_pose.orientation.y, current_robot_pose.orientation.z, current_robot_pose.orientation.w);
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    utils::Angle sfmRobotYaw = utils::Angle::fromRadian(yaw);

    // TODO GET NEW ROBOT POSITION AND MOVE
    double relativeAngle = atan2(sim->getAgentPosition(sim->getNumAgents() - 1).y() - current_robot_pose.position.y, sim->getAgentPosition(sim->getNumAgents() - 1).x() - current_robot_pose.position.x);

    std::cout << sim->getAgentPosition(sim->getNumAgents() - 1) << std::endl;

    double distance_to_goal = std::sqrt(std::pow(goal_x - current_robot_pose.position.x, 2) + std::pow(goal_y - current_robot_pose.position.y, 2));

    std::cout << "distance_to_goal = " << distance_to_goal << std::endl;

    if (distance_to_goal < 0.1)
    {
        goal_reached_ = true;
        stopRobot();
    }
    else
    {

        double angle_error = normalizeAngle(relativeAngle - yaw);

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

    delete sim;
}

