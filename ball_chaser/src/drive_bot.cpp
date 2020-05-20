#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool driveRequestCallback(ball_chaser::DriveToTarget::Request& request,
                          ball_chaser::DriveToTarget::Response& response) 
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = request.linear_x;
    cmd_vel.angular.z = request.angular_z;

    motor_command_publisher.publish(cmd_vel);

    response.msg_feedback = "Sent Linear X = " + 
                             std::to_string(request.linear_x) + 
                            " ; Angular Z = " + 
                            std::to_string(request.angular_z);

    ROS_INFO_STREAM(response.msg_feedback);

    return true;

}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", driveRequestCallback);

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}