//
// Created by alg on 07/04/21.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <fstream>

int main(int argc, char** argv){
    cv::Mat current_image = cv::imread("/home/alg/Pictures/white_bg_aruco5.png");

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > marker_corners, rejected_candidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(current_image, dictionary, marker_corners, markerIds, parameters, rejected_candidates);

    //filestream
    std::ofstream fout("/home/alg/marker5_corners.txt", std::ios::out);

    for(int i =0; i< marker_corners.size(); i++){
        fout<<marker_corners[i][0].x<<","<<marker_corners[i][0].y<<"\n";
        fout<<marker_corners[i][1].x<<","<<marker_corners[i][1].y<<"\n";
        fout<<marker_corners[i][2].x<<","<<marker_corners[i][2].y<<"\n";
        fout<<marker_corners[i][3].x<<","<<marker_corners[i][3].y<<"\n";
    }

    fout.close();

    return 0;
}