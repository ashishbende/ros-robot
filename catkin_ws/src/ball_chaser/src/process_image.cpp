#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

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

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // to decide where we need to turn, we can find the offset between the center of ball 
    // and center of camera image.

    float lin_x;
    float ang_z;

    int rows = img.height;
    int steps = img.step; // img.height?
    int image_center = steps/2.0;

    float offset = 0;
    int white_pix_count = 0;

    // loop through image pixels from top to bottom, and column/step wise
    for (int i = 0; i < rows ; ++i) {
        for (int j = 0; j < steps; ++j) {
            if (img.data[i * steps + j] == white_pixel) {
                offset += j - image_center;
                white_pix_count++;
            }
        }
    }

    float avg_offset = offset/white_pix_count;
    
    ROS_INFO("%d whitel pixels found. average  is %f", white_pix_count, avg_offset);

    // when no white pixel found
    /*keep looking for white ball pixel by rotating right?
    if(white_pix_count == 0){
        lin_x = 0.0;
        ang_z = -0.5;
    }*/
    if(white_pix_count == 0){ // stop 
        lin_x = 0.0;
        ang_z = 0.0;
    }else{
        lin_x = 0.1;
        // rough offset estimate 
        ang_z = -4.0 * avg_offset / image_center;
    }

    drive_robot(lin_x, ang_z);
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