//
// Created by alg on 4/9/21.
//

#ifndef CAMERA_PROJECTOR_UTILS_PROJECTIONS_H
#define CAMERA_PROJECTOR_UTILS_PROJECTIONS_H
#include <fstream>

void writeCorners2File(cv::Mat corners, std::string corner_file_name){
    std::ofstream fout(corner_file_name, std::ios::out);
    std::cout<<"writing corner to file";
    std::cout<<corners<<std::endl;
    for(int i =0; i< corners.rows; i++){
        fout<<corners.at<float_t>(i, 0)<<","<<corners.at<float_t>(i, 1)<<"\n";
    }
    fout.close();

}

void makeImgWhite(cv::Mat& img){
    for(int i =0; i< img.rows; i++)
        for(int j= 0; j< img.cols; j++)
            img.at<uchar>(i,j) = 255;
}
void makeImgBlack(cv::Mat& img){
    for(int i =0; i< img.rows; i++)
        for(int j= 0; j< img.cols; j++)
            img.at<uchar>(i,j) = 0;
}

void addImages(cv::Mat& bg, cv::Mat img){
    int start_loc_x = 250, start_loc_y = 250;
    for(int i =0; i< img.rows; i++){
        for(int j=0; j<img.cols; j++){

            bg.at<uchar>(i + start_loc_x,j + start_loc_y) = img.at<uchar>(i,j);
        }
    }

}

void showImgFS(std::string win_name, cv::Mat img){
    cv::namedWindow(win_name, CV_WINDOW_NORMAL);
    cv::setWindowProperty(win_name, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    cv::imshow(win_name, img);
    cv::waitKey(0);


}

#endif //CAMERA_PROJECTOR_UTILS_PROJECTIONS_H
