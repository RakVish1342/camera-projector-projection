//
// Created by alg on 4/9/21.
//
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "utils_projections.h"
//object start points
/* start location        object
 * (0, 0) =  (0,0)
 * (250, 400) = (3.6, 5.7)
 * (250, 1000) = (3.6, 5.7 + 8.4 = 14.1)
 * (500, 1000) = (7.2, 14.1)
 *
 * */
std::string aruco_image_path = "/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/checkerboard_cropped.png";
std::string corner_file_name="/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/temmp.txt";
const double scale = 0.2;
int main(){
    //load the checkerboard image
    cv::Mat checkerboard_img = cv::imread(aruco_image_path);
    cv::cvtColor(checkerboard_img, checkerboard_img, CV_BGR2GRAY);

    //resize image

    cv::resize(checkerboard_img, checkerboard_img, cv::Size(), scale, scale);
    std::cout<<"img size: "<<checkerboard_img.rows<<" "<<checkerboard_img.cols<<std::endl;

    //white background image
    cv::Mat background_white_img = cv::Mat(1080, 1920, CV_8UC1);
    makeImgWhite(background_white_img);
    //cv::cvtColor(background_white_img, background_white_img, CV_GRAY2BGR);
    std::cout<<"bg size: "<<background_white_img.rows<<" "<<background_white_img.cols<<"\n";
    //cv::cvtColor(background_white_img, main_img, CV_BGR2GRAY);

    //add the   checkerboard to the white image
    addImages(background_white_img, checkerboard_img);

    //detect corners
    cv::Mat corners;
    if(cv::findChessboardCorners(background_white_img, cv::Size(8, 6), corners ))
        std::cout<<"corners_written"<<std::endl;
    cv::drawChessboardCorners(background_white_img, cv::Size(8, 6), corners, true);
    cv::imshow("Detected Corners", background_white_img );
    cv::waitKey(0);

    //send the corners to a file
    writeCorners2File(corners, corner_file_name);

    showImgFS("CheckerBoard", background_white_img);


}

