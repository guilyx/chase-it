#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"

ros::ServiceClient srv_client;

typedef struct {
    int height = -1;
    int step = -1;
} Pixel;

void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!srv_client.call(srv)) {
        ROS_ERROR("Failed to call service");
    }
}

void imageProcessCallback(const sensor_msgs::Image img)
{
    Pixel white_pixel;

    // Detect white pixel
    for (std::size_t i = 0 ; i < img.height * img.step ; i++)
    {
        if (img.data[i] == 255) 
        {
            white_pixel.height = i / img.step;
            white_pixel.step = i % img.step;
            break;
        }
    }

    // Define area of white pixel
    float lin_x = .0;
    float ang_z = .0;

    if (white_pixel.step < img.step * .3 && white_pixel.step != -1)
    {
        ang_z = 0.2;
    } else if (white_pixel.step > img.step * .7 && white_pixel.step != -1)
    {
        ang_z = -0.2;
    } else if (white_pixel.step >= img.step * .3 && white_pixel.step <= img.step * .7 && white_pixel.step != -1)
    {
        lin_x = 0.5;
    }

    if (white_pixel.step != -1) {
        ROS_INFO_STREAM("White Ball detected, driving towards it...");
    } else {
        ROS_INFO_STREAM("White Ball out of sight, stopping the robot.");
        ang_z = 0.0;
        lin_x = 0.0;
    }

    // send driving request to robot
    drive_robot(lin_x, ang_z);
}



int main(int argc, char** argv) 
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle nh;

    srv_client = nh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber imSub = nh.subscribe("/camera/rgb/image_raw", 10, imageProcessCallback);
    ros::spin();

    return 0;
}