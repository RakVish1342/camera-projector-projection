//
// Created by alg on 4/19/21.
//

#ifndef CAMERA_PROJECTOR_PIXELCALCULATIONS_H
#define CAMERA_PROJECTOR_PIXELCALCULATIONS_H
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>




//Eigen::Matrix3d eigen_k_camera, eigen_k_projector;



//using opencv project points
cv::Point2d calculatePixels(Eigen::Vector3d corner){
    cv::Mat_<double> cv_k_camera(3,3);
    cv::Mat_<double> cv_k_projector(3,3), cv_d_camera(1,5), cv_d_projector(1,5);
    cv_k_camera << 614.62353515625, 0.0, 327.8028869628906,
            0.0, 614.45849609375, 238.79359436035156,
            0.0, 0.0, 1.0;

    cv_k_projector << 1329.73, 0.0, 420.59,
                      0.0, 1322.18, 395.68,
                      0.0, 0.0, 1.0;
    cv_d_camera <<  0.17198306778548356, -0.15695033321750945, -0.0065613334239603060, 0.0035590493094028669, 0.0;

    cv_d_projector << 0.56581469574959975, -3.8373751872066872, -0.017805365171805934, -0.025716309863278747, 0.0;

    Eigen::Matrix3d R_proj_cam;
    R_proj_cam <<0.99957512114540603, -0.023479623897468518, 0.017270913374231957,
                 0.020922515744415292, 0.99052929754749930, 0.13569804359305362,
                -0.020293484719768884, -0.13527903740672839, 0.99059969539478332;

    Eigen::Vector3d T_proj_cam;
    T_proj_cam <<  0.10171584394938355, -0.18582068182488356, 0.12325087629628882;
    Eigen::Vector3d T_proj_corner;

    /** corner in projector frame **/
    T_proj_corner = R_proj_cam*corner + T_proj_cam;
    cv::Mat_<double> cv_t_proj_corner(1, 3);
    cv_t_proj_corner <<   T_proj_cam.x() , T_proj_cam.y(), T_proj_cam.z();

    //identity for R
    cv::Mat_<double> rvec_identity(3,3), tvec_0(1,3);
    rvec_identity << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;
    tvec_0 << 0, 0, 0;
    cv::Mat cv_pixel_coordinates;
    cv::projectPoints(cv_t_proj_corner,
                      rvec_identity, tvec_0,
                      cv_k_projector, cv_d_projector,
                      cv_pixel_coordinates);
    cv::Point2d corner_point;
    //TODO: SOURCE OF ERROR. CHECK
    corner_point.x = cv_pixel_coordinates.at<double>(0,0);
    corner_point.y = cv_pixel_coordinates.at<double>(0,1);
    return corner_point;
}

void calculatePixelsInProjectorPlane(Eigen::Vector3d corner_tr,Eigen::Vector3d corner_tl,
                                     Eigen::Vector3d corner_br,Eigen::Vector3d corner_bl,
                                     cv::Point2d& px_tl, cv::Point2d& px_tr,
                                     cv::Point2d& px_br, cv::Point2d& px_bl){

    //get the pixel coordinates using opencv
    px_tl = calculatePixels(corner_tl);
    px_tr = calculatePixels(corner_tr);
    px_br = calculatePixels(corner_br);
    px_bl = calculatePixels(corner_bl);


}

void drawAndShowCorners(cv::Point2d  px_tl, cv::Point2d  px_tr,
                        cv::Point2d  px_br, cv::Point2d  px_bl){

    //get image
    cv::Mat img = cv::imread("/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/white_bg_aruco1.png");
    cv::circle(img, px_tl, 15, (255,0,0), 5 );
    cv::circle(img, px_tr, 15, (0,255,0), 5 );
    cv::circle(img, px_br, 15, (0,0,255), 5 );
    cv::circle(img, px_bl, 15, (255,0,255), 5 );

    cv::imshow("ImageWithCorners", img);
    cv::waitKey(5);
}

#endif //CAMERA_PROJECTOR_PIXELCALCULATIONS_H
