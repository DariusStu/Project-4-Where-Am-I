#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

using namespace std;

// Contributer: Darius Stu

// Set ROS publisher
ros::Publisher motor_command_publisher;

/*
 * Callback function that executes whenever a drive_bot service is requested. This function publishes
 * the requested linear x and angular velocities to the robot wheel joints. After publsihing the 
 * requested velocities, a message feedback is returned with the requested wheel velocities.
 */
bool handle_drive_request_callback(ball_chaser::DriveToTarget::Request& request, ball_chaser::DriveToTarget::Response& response)
{
    // ROS information message
    ROS_INFO("DriveToTarget service request recieved.");
    
    // Motor command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
  
    // Set wheel velocities and then publish the message using ros publisher
    motor_command.linear.x = request.linear_x;
    motor_command.angular.z = request.angular_z;
    motor_command_publisher.publish(motor_command);

    // Setting up the response message
    response.msg_feedback = "The velocities were set to the follwing: linear x: " + to_string((double)motor_command.linear.x) + ", and angular_z: " + to_string((double)motor_command.angular.z);
    ROS_INFO_STREAM(response.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize ros Node and set up node handle
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle nh;
    
    // Set up the ros publisher on the cmd_vel topic
    motor_command_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to the ball_chaser/command topic and read the requested wheel velocities 
    ros::ServiceServer service = nh.advertiseService("/ball_chaser/command_robot", handle_drive_request_callback);

    // call to ROS spin 
    ros::spin();

return 0;
}
