#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

// Define the pick-up and drop-off coordinates
#define PICKUP_X 4.0     // pick-up x coordinate
#define PICKUP_Y -5.0    // pick-up y coordinate
#define DROPOFF_X -6.0   // drop-off x coordinate
#define DROPOFF_Y -5.0   // drop-off y coordinate

// Current robot pose
double pose_x = 0.0;
double pose_y = 0.0;

// Define the threshold distance to consider the robot has reached a zone
const double THRESHOLD = 0.3;

// Enum to represent the current state of the robot
enum State {
    PICKUP,      // Moving to pick-up zone
    TRAVEL,      // Traveling to drop-off zone
    DROPOFF      // Dropped off
};

// Current state
State current_state = PICKUP;

// Publisher for the marker
ros::Publisher marker_pub;

// Function to update the robot's current pose based on odometry data
void get_current_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
}

// Function to calculate the Euclidean distance between two points
double calc_distance(double goal_x, double goal_y)
{
    double delta_x = goal_x - pose_x;
    double delta_y = goal_y - pose_y;
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1); // 1 Hz

    // Initialize the publisher for visualization markers
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Subscribe to the odometry topic to get the robot's current pose
    ros::Subscriber odom_sub = n.subscribe("/odom", 10, get_current_pose);

    // Wait until there is at least one subscriber to the marker topic
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        ros::Duration(1.0).sleep();
    }

    // Initialize the marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "home_service_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker (only 2D x and y; z is 0)
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker (size of the cube)
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color of the marker (green)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Ensure the marker never auto-deletes
    marker.lifetime = ros::Duration();

    ROS_INFO("Publishing marker at pick-up zone.");

    // Initially set the marker at the pick-up zone
    marker.pose.position.x = PICKUP_X;
    marker.pose.position.y = PICKUP_Y;
    marker.header.stamp = ros::Time::now(); // Update timestamp
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);

    while (ros::ok())
    {
        ros::spinOnce(); // Process incoming messages

        switch (current_state)
        {
            case PICKUP:
            {
                // Check if the robot has reached the pick-up zone
                double distance = calc_distance(PICKUP_X, PICKUP_Y);
                if (distance < THRESHOLD)
                {
                    ROS_INFO("Robot reached the pick-up zone.");

                   

                    // Simulate a pick-up delay
                    ROS_INFO("Simulating pick-up for 5 seconds...");
                    ros::Duration(5.0).sleep();
                    
                     // Hide the marker to simulate pick-up
                    marker.action = visualization_msgs::Marker::DELETE;
                    marker.header.stamp = ros::Time::now(); // Update timestamp
                    marker_pub.publish(marker);

                    // Transition to TRAVEL state
                    current_state = TRAVEL;
                }
                break;
            }

            case TRAVEL:
            {
                // Update the marker's position to drop-off zone
                marker.pose.position.x = DROPOFF_X;
                marker.pose.position.y = DROPOFF_Y;

                // Show the marker at drop-off zone to simulate carrying the package
                ROS_INFO("Moving to drop-off zone...");
                

                // Transition to DROPOFF state
                current_state = DROPOFF;
                break;
            }

            case DROPOFF:
            {
                // Check if the robot has reached the drop-off zone
                double distance = calc_distance(DROPOFF_X, DROPOFF_Y);
                if (distance < THRESHOLD)
                {
                    ROS_INFO("Robot reached the drop-off zone.");

                    // Simulate a drop-off delay
                    ROS_INFO("Simulating drop-off for 3 seconds...");
                    ros::Duration(3.0).sleep();
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.header.stamp = ros::Time::now(); // Update timestamp
                    marker_pub.publish(marker);
                    
                }
                break;
            }

            default:
                break;
        }

        r.sleep(); // Sleep to maintain the loop rate
    }

    return 0;
}

