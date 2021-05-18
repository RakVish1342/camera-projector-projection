#define NUM_FILES 233

#include <fstream>
#include <istream>
#include <ios>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


void readCorners(std::vector< std::pair<int, int> >& cornersList, const std::string& filePrefix, std::string basePath)
{
    std::string fileSuffix = ".txt";
    std::string fileName;
    for(int i = 1; i <= NUM_FILES; ++i)
    {   
        fileName = filePrefix + std::to_string(i) + fileSuffix;
        std::cout << fileName << std::endl;

        // https://www.geeksforgeeks.org/csv-file-management-using-c/
        // https://stackoverflow.com/questions/5605125/why-is-iostreameof-inside-a-loop-condition-i-e-while-stream-eof-cons
        // OR Use rangeImageVisualization.cpp from TreeMapping if is a space separated/delimited filed
        std::string line;
        std::ifstream cornersFile(basePath + fileName);
        if (cornersFile.is_open())
        {   
            while (std::getline(cornersFile, line))
            {
                // cout << line << '\n';
                std::string x, y;
                std::stringstream ss(line);
                std::getline(ss, x, ',');
                std::getline(ss, y, ',');
                std::cout << x << ", " << y << std::endl;
                cornersList.push_back( {std::stoi(x), std::stoi(y)} );
            }
        }
        {
            std::cout << "Error or Empty File." << std::endl;
        }

    }

}


struct CostFunctor
{
    CostFunctor(double xc, double yc, double xp, double yp) : xc(xc), yc(yc), xp(xp), yp(yp) {}

    template <typename T>
    bool operator()(const T *const h1,
                    const T *const h2, 
                    const T *const h3,
                    const T *const h4,
                    const T *const h5,
                    const T *const h6,
                    const T *const h7,
                    const T *const h8,
                    const T *const h9,
                    T *residual) const
    {
        // const T *const xcomp = xp - ( H[0]*xc + H[2]*yc + H[3] ); 
        // const T *const ycomp = yp - ( H[3]*xc + H[4]*yc + H[5] ); 
        // const T *const zcomp =  1 - ( H[6]*xc + H[7]*yc + H[8] ); 

        // const T *const xcomp = xp - ( h1[0]*xc + h2[0]*yc + h3[0] ); 
        // const T *const ycomp = yp - ( h4[0]*xc + h5[0]*yc + h6[0] ); 
        // const T *const zcomp =  1 - ( h7[0]*xc + h8[0]*yc + h9[0] ); 
        // residual[0] = xcomp*xcomp + ycomp*ycomp + zcomp*zcomp;

        residual[0] =   (xp - (h1[0]*xc + h2[0]*yc + h3[0])) * (xp - (h1[0]*xc + h2[0]*yc + h3[0])) + 
                        (yp - (h4[0]*xc + h5[0]*yc + h6[0])) * (yp - (h4[0]*xc + h5[0]*yc + h6[0])) +
                        (1.0 - (h7[0]*xc + h8[0]*yc + h9[0])) * (1.0 - (h7[0]*xc + h8[0]*yc + h9[0])); // Ensure all constants are of double (decimal) and not int type. Else "Incompatible binary operation "*" between int and const double" will appear.

        return true;
    }

private:
    const double xc;
    const double yc;
    const double xp;
    const double yp;
};

void optimizeHomographyMatrix(const std::vector<std::pair<int, int>>& cameraCorners, 
                            const std::vector<std::pair<int, int>>& projectorCorners, 
                            std::vector<double> H)
{
    // google::InitGoogleLogging(argv[0]);


    //TODO: Any good initialization value inferrable?
    //TODO: Send homography matrix at once as a vector/EigenVector/EigenMatrix instead of individual components
    // double h1, h2, h3, h4, h5, h6, h7, h8, h9; // If int h1, h2...h9 is used, compilation error obtained: "no matching function for call to ‘ceres::Problem::AddResidualBlock(ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1>*, NULL, int*, int*, int*, int*, int*, int*, int*, int*, int*)’ "
    double h2 = 0.0, h3 = 0.0, h4 = 0.0, h6 = 0.0, h7 = 0.0, h8 = 0.0;
    double h1 = 1.0, h5 = 1.0, h9 = 1.0;

    std::cout << "BEFORE" << std::endl;
    std::cout << h1 << ", " << h2 << ", " << h3 << std::endl;
    std::cout << h4 << ", " << h5 << ", " << h6 << std::endl;
    std::cout << h7 << ", " << h8 << ", " << h9 << std::endl;

    Problem problem;
    for (int i = 0; i < cameraCorners.size(); ++i)
    {
        // problem.AddResidualBlock(
        //     new AutoDiffCostFunction<CostFunctor, 1, 9>( // Dimensions of: residual, H [h1, h2, h3; h4, h5, h6; h7, h8, h9]
        //         new CostFunctor(cameraCorners[i].first, cameraCorners[i].second, projectorCorners[i].first, projectorCorners[i].second)),
        //     NULL,
        //     &H);
        problem.AddResidualBlock(
            // new AutoDiffCostFunction<CostFunctor, 1, 9>( // Dimensions of: residual, H [h1, h2, h3; h4, h5, h6; h7, h8, h9]
            new AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1>( // Dimensions of: residual, H [h1, h2, h3; h4, h5, h6; h7, h8, h9]
                new CostFunctor(cameraCorners[i].first, cameraCorners[i].second, projectorCorners[i].first, projectorCorners[i].second)),
            NULL,
            &h1, &h2, &h3, &h4, &h5, &h6, &h7, &h8, &h9);

        H = {h1, h2, h3, h4, h5, h6, h7, h8, h9 };

    }

    Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    std::cout << "AFTER" << std::endl;
    std::cout << h1 << ", " << h2 << ", " << h3 << std::endl;
    std::cout << h4 << ", " << h5 << ", " << h6 << std::endl;
    std::cout << h7 << ", " << h8 << ", " << h9 << std::endl;


}


int main(int argc, char **argv)
{
    std::string basePath = "/home/rxth/catkin_ws/src/CameraProjectorProjection/camera_projector/data/corners/";
    std::vector< std::pair<int, int> > cameraCorners;
    std::vector< std::pair<int, int> > projectorCorners;
    readCorners(cameraCorners, "corners_camera", basePath);
    readCorners(projectorCorners, "corners_projector", basePath);

    std::cout << "---" << std::endl;
    std::cout << cameraCorners.size() << ", " << projectorCorners.size() << std::endl;

    // std::vector<double> H (9, 0.0);  // Flattened 3x3 homogrphy matrix H
    std::vector<double> H;  // Flattened 3x3 homogrphy matrix H
    if(cameraCorners.size() == projectorCorners.size())
    {
        optimizeHomographyMatrix(cameraCorners, projectorCorners, H);
    }
    else
    {
        std::cout << "Unequal size." << std::endl;
    }
    
    return 0;

}