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
#include <ros/ros.h>
#include "homography_optimization_utils.h"


std::string image_topic_name = "/camera/color/image_raw";
bool projector_state = true;
bool camera_state = false;
cv::Mat realsense_image;
std::vector<std::vector<cv::Point2f>> projector_marker_corners;
std::vector<std::vector<cv::Point2f>> camera_marker_corners;
std::string projector_corners_filename = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/corners/corners_projector";
std::string camera_corners_filename = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/corners/corners_camera";
int fileIdx=1;

void realsenseImageCallback(const sensor_msgs::ImageConstPtr& msg){
    std::ofstream fout_camera(camera_corners_filename+std::to_string(fileIdx)+".txt", std::ios::out);    
    std::ofstream fout_projector(projector_corners_filename+std::to_string(fileIdx)+".txt", std::ios::out);


    std::cout << "IMAGE CB" << std::endl;

    // Save msg to variable
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


    // RealSense Processing //
    //detect corners in that image
    detectArucoCorners(realsense_image, camera_marker_corners); 
    std::cout << "CAM" << camera_marker_corners.size() << std::endl;

    //TODO: Check if these corners are same as the last ones
    //if corners detected in both, and is a new image (projector_state != image_state), save the corners
    if( (projector_marker_corners.size()>0) && (camera_marker_corners.size()>0) && (projector_state != camera_state) ){

        for(int i =0; i< projector_marker_corners.size(); i++){

            std::cout<<"Wrtubg ti fuke\n"; 
            fout_projector<<projector_marker_corners[i][0].x<<","<<projector_marker_corners[i][0].y<<"\n";
            fout_projector<<projector_marker_corners[i][1].x<<","<<projector_marker_corners[i][1].y<<"\n";
            fout_projector<<projector_marker_corners[i][2].x<<","<<projector_marker_corners[i][2].y<<"\n";
            fout_projector<<projector_marker_corners[i][3].x<<","<<projector_marker_corners[i][3].y<<"\n";
        }

        for(int i =0; i< camera_marker_corners.size(); i++){
             std::cout<<"Wrtubg ti fukee\n";
            fout_camera<<camera_marker_corners[i][0].x<<","<<camera_marker_corners[i][0].y<<"\n";
            fout_camera<<camera_marker_corners[i][1].x<<","<<camera_marker_corners[i][1].y<<"\n";
            fout_camera<<camera_marker_corners[i][2].x<<","<<camera_marker_corners[i][2].y<<"\n";
            fout_camera<<camera_marker_corners[i][3].x<<","<<camera_marker_corners[i][3].y<<"\n";
         
        }

            std::cout <<"==========="<<std::endl;
            std::cout<<projector_marker_corners[0][0].x<<","<<projector_marker_corners[0][0].y<<"\n";
            std::cout<<projector_marker_corners[0][1].x<<","<<projector_marker_corners[0][1].y<<"\n";
            std::cout<<projector_marker_corners[0][2].x<<","<<projector_marker_corners[0][2].y<<"\n";
            std::cout<<projector_marker_corners[0][3].x<<","<<projector_marker_corners[0][3].y<<"\n";
            std::cout <<"---"<<std::endl;
            std::cout<<camera_marker_corners[0][0].x<<","<<camera_marker_corners[0][0].y<<"\n";
            std::cout<<camera_marker_corners[0][1].x<<","<<camera_marker_corners[0][1].y<<"\n";
            std::cout<<camera_marker_corners[0][2].x<<","<<camera_marker_corners[0][2].y<<"\n";
            std::cout<<camera_marker_corners[0][3].x<<","<<camera_marker_corners[0][3].y<<"\n";   
            std::cout <<"---"<<std::endl;

        camera_state = projector_state;
        projector_marker_corners.clear();
        camera_marker_corners.clear();
        fileIdx += 1;
    }

}


int main(int argc, char** argv){

    ros::init(argc, argv, "homography_optimization_viz");
    std::cout << "Running Node: homography_optimization_viz..." << std::endl;
    ros::NodeHandle nh;

    //get ros image realsenses
    ros::Subscriber sub_image = nh.subscribe(image_topic_name, 1000, realsenseImageCallback);

    ros::Rate rate(1);

    // Start the opencv windowing system 
    //TODO: Without a test image being displayed, the full screen windowing system does not work. Fix this.
    cv::Mat tmpImg = cv::Mat(100, 100, CV_8UC1, cv::Scalar(0));

    int ctr = 0;
    int ctr_j = 0; 
    while(ros::ok()){
        if((ctr%7) == 0) { 
            ctr_j++; 
            ctr = 0;
        }
        if(ctr_j> 36) break; 

        std::cout << "CTR:" << ctr << std::endl;

        //make an image with white background and aruco marker
        cv::Mat projector_aruco_img;
        
        makeArucoImage(projector_aruco_img, 100+ctr*50, 100 + ctr_j*50);

        //detect aruco markers in this image
        detectArucoCorners(projector_aruco_img, projector_marker_corners);
        std::cout << "PROJ" << projector_marker_corners[0].size() << std::endl;

        //show aruco marker in full screen
        showImgFS("Projector Screen", projector_aruco_img, 2000);
        ros::spinOnce(); // capture image

        // reset flag to indicate new image in next iteration
        projector_state = !projector_state;
        ctr += 1;
        rate.sleep();
    }

 //   fout_projector.close(); 
   // fout_camera.close(); 

    return 0;
}