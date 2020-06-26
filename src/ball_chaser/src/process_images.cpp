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
    /*
     * Setup variables 
     * imageHeight - image height, number of rows
     * imageStep - Full row length in bytes
     */
    int white_pixel = 255;
    int imageHeight = img.height;
    int imageStep = img.step;

    // intialize linear x velocity and angular z velocity variables
    float x = 0.0;
    float z = 0.0;

    /*
     * Loop through each pixel in image. Use the image height to loop from top to  
     * bottom, and the image step to loop left to right. If the pixel is bright (white ball) pixels
     * then increase the total count for white pixels. The variable offsetAccumlated is used to 
     * compute the offset from the center of the ball to the center of the picture.
     */
    float offsetAccumlated = 0.0;
    int totalCount = 0.0;
    for (int heightIndex = 0; heightIndex < imageHeight; heightIndex++)
    {
        for (int stepIndex = 0; stepIndex < imageStep; stepIndex++)
        {
            if (img.data[heightIndex * imageStep + stepIndex] == white_pixel)
            {
                offsetAccumlated += stepIndex - imageStep / 2.0;
                totalCount++;
            }
        }
    
    }

    /*
     * If no white pixels are detected then the car will stop (no velocity). 
     * Else the liner x-velocity will be 0.1 m/s and the angular z velocity
     * will be the average offset normailzed between -1.0 to 1.0. 
     * We will have a turning coefficient = -4 (this number can be changed and was 
     * determined through testing).
     */  
    int turningCoeff = -4;
    
    if(0 == totalCount)
    {
        // no velocity
        x = 0.0;
        z = 0.0;
    }
    else
    {
        x = 0.1;
        z = turningCoeff * offsetAccumlated / totalCount / (imageStep / 2.0);
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
