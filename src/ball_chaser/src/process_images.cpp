#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

using namespace std;

// Contributer: Darius Stu

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_bot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving to target!");

    // Request service with velocities
    ball_chaser::DriveToTarget service;
    service.request.linear_x = lin_x;
    service.request.angular_z = ang_z;

    if(!client.call(service))
    {
        ROS_ERROR("Failed to call service DriveToTarget. Yikes!");
    }
}

void process_image_callback(const sensor_msgs::Image img)
{
    // Only scan the middle third of the image, ball is ground level with robot
    int startScan = img.data.size() / 3;
    int endScan = img.data.size() *2 / 3;

    // intialize linear x velocity and angular z velocity variables
    float x = 0.0;
    float z = 0.0;
    int countOfWhitePixels = 0.0;    
    int xPosSum = 0;
    for (int pixelIndex = startScan; pixelIndex+2 < endScan; pixelIndex+=3)
    {
        //Setting up RGB (Red, Green, Blue) channels
        int channelRed = img.data[pixelIndex];
        int channelGreen = img.data[pixelIndex+1];
        int channelBlue = img.data[pixelIndex+2]; 
        
        // A white pixel will have all RGB values of 255. 
        if (channelRed == 255 && channelGreen == 255 && channelBlue == 255)
        {
            int xPosition = (pixelIndex % (img.width *3)) / 3;
            xPosSum += xPosition;
            countOfWhitePixels += 1;
        }
    }

    // If no white pixels are detected then the car will stop (no velocity). 
    
    if (0 == countOfWhitePixels)
    {
        // no velocity
        x = 0.0;
        z = 0.0;
    }
    else
    {
        int meanX = xPosSum / countOfWhitePixels;
        if (meanX < img.width /3)
        {
            x = 0.5;
            z = 0.5;
        }
        else if (meanX > img.width * 2 / 3)
        {
            x = 0.5;
            z = -0.5;
        }
        else
        {
            // Drive straight
            x = 0.5;
            z = 0.0;
        }
    }

    // Send service request
    drive_bot(x,z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a node handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle pI;

    // Define a client service capable of requesting services from command_robot
    client = pI.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber pI_subscriber = pI.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Call to ROS spin function
    ros::spin();
    
    return 0;
}
