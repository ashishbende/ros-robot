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
    int red_channel = 0;
    int green_channel = 0;
    int blue_channel = 0;
    
    int mat_col_index = 0;
    int l_pixels = 0;
    int m_pixels = 0;
    int r_pixels = 0;


    // each step comprise of r,g,b chnls
    for(int i=0; i< img.height*img.step; i+=3){

        red_channel = img.data[i];
        green_channel = img.data[i+1];
        blue_channel = img.data[i+2];
        mat_col_index = i % (img.step)/3;

        // all three channels have white pixels
        if(red_channel == white_pixel && green_channel == white_pixel
           && blue_channel == white_pixel){
               if(mat_col_index <=265){
                   l_pixels++;   
               }
               if(mat_col_index > 265 && mat_col_index <=533){
                   m_pixels++;
               }
               if(mat_col_index > 533){
                   r_pixels++;
               }
        }
    }

    // 
    int direction = (l_pixels > m_pixels) 
                        ?  (l_pixels > r_pixels ? l_pixels : r_pixels)
                        :  (m_pixels > r_pixels ? m_pixels : r_pixels);
    
    // Depending on the white ball position, call the drive_bot function and pass velocities to it.
    // Request a stop when there's no white ball seen by the camera.
    ROS_INFO("Direction = %d, and pixels =[%d, %d, %d]", direction, l_pixels, m_pixels, r_pixels);
    if(direction == l_pixels){
        /* drive toward left */
        drive_robot(0.0, -0.5);
    }
    
    if(direction == m_pixels){
        /* drive forward */
        drive_robot(0.5, 0.0);
    }
    
    if(direction == r_pixels){
        /* drive toward right */
        drive_robot(0.0, 0.5);
    }
    
    if(direction == 0.0){
        /* otherwise stop */
        drive_robot(0.0, 0.0);
    }
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