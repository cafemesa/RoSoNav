#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ActorPositionPublisher {
public:
    ActorPositionPublisher() {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscribe to Gazebo model states
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, &ActorPositionPublisher::modelStatesCallback, this);

        // Publish point cloud
        actors_positions_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/actors_positions", 1);
        robot_position_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/robot_position", 1);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        // Extract positions of actors and mobile base
        pcl::PointCloud<pcl::PointXYZ> point_cloud_actors, point_cloud_robot;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i].find("actor") != std::string::npos && msg->pose[i].position.z > 0.5) {
                pcl::PointXYZ point;
                point.x = msg->pose[i].position.x;
                point.y = msg->pose[i].position.y;
                point.z = msg->pose[i].position.z;
                point_cloud_actors.push_back(point);
            }
        }


        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "mobile_base") {
                pcl::PointXYZ point;
                point.x = msg->pose[i].position.x;
                point.y = msg->pose[i].position.y;
                point.z = msg->pose[i].position.z;
                point_cloud_robot.push_back(point);
            }
        }

        // Convert point cloud to ROS message
        sensor_msgs::PointCloud2 ros_cloud_actors;
        pcl::toROSMsg(point_cloud_actors, ros_cloud_actors);
        ros_cloud_actors.header.frame_id = "map"; // Modify the frame_id if needed


        sensor_msgs::PointCloud2 ros_cloud_robot;
        pcl::toROSMsg(point_cloud_robot, ros_cloud_robot);
        ros_cloud_robot.header.frame_id = "map"; // Modify the frame_id if needed

        // Publish the point cloud
        actors_positions_pub_.publish(ros_cloud_actors);
        robot_position_pub_.publish(ros_cloud_robot);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber model_states_sub_;
    ros::Publisher actors_positions_pub_, robot_position_pub_;
};

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "position_publisher");

    // Create an instance of ActorPositionPublisher
    ActorPositionPublisher actor_position_publisher;

    // Spin
    ros::spin();

    return 0;
}
