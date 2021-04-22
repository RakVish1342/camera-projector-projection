//
// Created by alg on 4/19/21.
//

#ifndef CAMERA_PROJECTOR_HOMOGRAPHY_OPTIMIZATION_UTILS_H
#define CAMERA_PROJECTOR_HOMOGRAPHY_OPTIMIZATION_UTILS_H

#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "utils_projections.h"

std::string aruco_image_path = "/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/marker.png";
std::string corner_file_name="/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/aruco_corners.txt";
const double scale = 1;

void makeArucoImage (cv::Mat& img){
    //load the checkerboard image
    cv::Mat aruco_img = cv::imread(aruco_image_path);
    std::cout << "img size: " << aruco_img.rows << " " << aruco_img.cols << std::endl;

    //white background image
    cv::Mat background_white_img = cv::Mat(1080, 1920, CV_8UC1);
    makeImgWhite(background_white_img);
    std::cout<<"bg size: "<<background_white_img.rows<<" "<<background_white_img.cols<<"\n";

    //add the   checkerboard to the white image
    addImages(background_white_img, aruco_img);
    img = background_white_img.clone();

}
void detectArucoCorners(cv::Mat current_image, std::vector<std::vector<cv::Point2f> >& marker_corners){

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(current_image, dictionary, marker_corners, markerIds, parameters, rejected_candidates);

}



#endif //CAMERA_PROJECTOR_HOMOGRAPHY_OPTIMIZATION_UTILS_H
