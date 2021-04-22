//
// Created by alg on 4/19/21.
//

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include<ros/ros.h>
#include "homography_optimization_utils.h"


std::string image_topic_name = "/camera/color/image_raw";
bool new_image = false;
cv::Mat realsense_image;

//corner file names
std::string projector_corners_filename = "/home/alg/projector_corners.txt";
std::string camera_corners_filename = "/home/alg/camera_corners.txt";

void realsenseImageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    realsense_image = cv_ptr->image;
    new_image = true;

}

int main(int argc, char** argv){

    ros::init(argc, argv, "homography_optimization_viz");
    ros::NodeHandle nh;

    //get ros image realsenses
    ros::Subscriber sub_image = nh.subscribe(image_topic_name, 1000, realsenseImageCallback);

    ros::Rate rate(20);

    std::ofstream fout_projector(projector_corners_filename, std::ios::out);
    std::ofstream fout_camera(camera_corners_filename, std::ios::out);


    while(ros::ok()){


        //make an image with white background and aruco marker
        cv::Mat projector_aruco_img;
        makeArucoImage(projector_aruco_img);

        //detect aruco markers in this image
        std::vector<std::vector<cv::Point2f> > projector_marker_corners;
        detectArucoCorners(projector_aruco_img, projector_marker_corners);

        //show aruco marker in full screen
        showImgFS("Projector Screen", projector_aruco_img);

        //time delay of 1s
        ros::Duration(1).sleep();


        /** RealSense Processing **/
        //detect corners in that image
        std::vector<std::vector<cv::Point2f> > camera_marker_corners;
        detectArucoCorners(realsense_image, camera_marker_corners );


        //TODO: Check if these corners are same as the last ones
        //if corners are detected add both sets of corners  to different files
        if(camera_marker_corners.size() > 0){

            for(int i =0; i< projector_marker_corners.size(); i++){
                fout_projector<<projector_marker_corners[i][0].x<<","<<projector_marker_corners[i][0].y<<"\n";
                fout_projector<<projector_marker_corners[i][1].x<<","<<projector_marker_corners[i][1].y<<"\n";
                fout_projector<<projector_marker_corners[i][2].x<<","<<projector_marker_corners[i][2].y<<"\n";
                fout_projector<<projector_marker_corners[i][3].x<<","<<projector_marker_corners[i][3].y<<"\n";
            }

            for(int i =0; i< camera_marker_corners.size(); i++){
                fout_camera<<camera_marker_corners[i][0].x<<","<<camera_marker_corners[i][0].y<<"\n";
                fout_camera<<camera_marker_corners[i][1].x<<","<<camera_marker_corners[i][1].y<<"\n";
                fout_camera<<camera_marker_corners[i][2].x<<","<<camera_marker_corners[i][2].y<<"\n";
                fout_camera<<camera_marker_corners[i][3].x<<","<<camera_marker_corners[i][3].y<<"\n";
            }

        }


        ros::spinOnce();
        rate.sleep();
    }

    fout_camera.close();
    fout_projector.close();
    return 0;
}