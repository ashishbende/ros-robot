#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include "ros/console.h"

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget drive_cmd;
    drive_cmd.request.linear_x = lin_x;
    drive_cmd.request.angular_z = ang_z;

    if(!client.call(drive_cmd)){
        ROS_ERROR("Client error. Unable to make drive robot call.");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    bool ball_found = false;
    
    for (int i=0; i < img.height * img.step; i += 3)
    {
        int r_ch = img.data[i];
        int g_ch = img.data[i+1];
        int b_ch = img.data[i+2];

        // 1. presence - when all three color channels are white
        // 2. position - position wrt to camera left,middle or right 
        if (!(r_ch^white_pixel) 
            && !(g_ch^white_pixel) 
            && !(b_ch^white_pixel))
        {
            int mat_col_index = i % img.step;
            // left col pixel, turn left
            if (mat_col_index < img.step/3){ drive_robot(0.5, 1); }
            // mid col pixel, go straight
            else if (mat_col_index < (img.step/3 * 2)){ drive_robot(0.5, 0); } 
            // right most col, turn right 
            else{ drive_robot(0.5, -1);}
            ball_found = true;
            break;
         }
    }

    if (!ball_found)
        drive_robot(0, 0);
    // else  // keep looking for white ball
    // {
    //     drive_robot(0.5, 1);
    // }   
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}