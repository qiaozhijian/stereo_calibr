//
// Created by qzj on 2020/6/27.
//

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include "util.h"
#include <stdio.h>
#include <yaml-cpp/yaml.h>

using namespace cv;
using namespace std;

template<typename T>
FileStorage &operator,(FileStorage &out, const T &data) {
    out << data;
    return out;
}

int main(int argc, char *argv[])
{
    // -d 参数，总文件夹，应该包括result，left，right三个子文件夹
    cv::CommandLineParser parser(argc, argv,
                                 "{{d|/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round2/cali|}{show|true|}{help||}");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    string root_path = parser.get<string>("d");
    string root_result_path = root_path + "/result/";
    createDirectory(root_result_path);

    //读入matlab保存的yaml
    YAML::Node fsSettings = YAML::LoadFile(root_result_path + "cali_mat.yaml");

    cv::Mat K_l, K_r, D_l, D_r,T_lr,R_lr;
    cv::Mat Rl, Rr, Pl, Pr, Q; //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
    cv::Size size;

    K_l = Mat::ones(3, 3, CV_64F);
    K_r = Mat::ones(3, 3, CV_64F);
    R_lr = Mat::ones(3, 3, CV_64F);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            K_l.row(i).col(j) = fsSettings["K1"][i][j].as<double>();
            K_r.row(i).col(j) = fsSettings["K2"][i][j].as<double>();
            R_lr.row(i).col(j) = fsSettings["rot"][i][j].as<double>();
        }

    D_l = Mat::ones(5, 1, CV_64F);
    D_r = Mat::ones(5, 1, CV_64F);
    for(int i=0;i<5;i++)
        for(int j=0;j<1;j++)
        {
            D_l.row(i).col(j) = fsSettings["D1"][i].as<double>();
            D_r.row(i).col(j) = fsSettings["D2"][i].as<double>();
        }

    size.height = fsSettings["size"][0].as<float>();
    size.width = fsSettings["size"][1].as<float>();

    T_lr = Mat::ones(3, 1, CV_64F);
    for(int i=0;i<3;i++)
        for(int j=0;j<1;j++)
        {
            T_lr.row(i).col(j) = fsSettings["trans"][i].as<double>();
        }

    cout << "finish input" << endl;

    int rows_l = size.height;
    int cols_l = size.width;
    int rows_r = size.height;
    int cols_r = size.width;

    cv::Size imageSize(cols_l,rows_l);
    //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域, 其内部的所有像素都有效
    cv::Rect validROIL;
    cv::Rect validROIR;
    //经过双目标定得到摄像头的各项参数后，采用OpenCV中的stereoRectify(立体校正)得到校正旋转矩阵R、投影矩阵P、重投影矩阵Q
    //flags-可选的标志有两种零或者 CV_CALIB_ZERO_DISPARITY ,如果设置 CV_CALIB_ZERO_DISPARITY 的话，该函数会让两幅校正后的图像的主点有相同的像素坐标。否则该函数会水平或垂直的移动图像，以使得其有用的范围最大
    //alpha-拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
    cout << "finish rectify" << endl;
    stereoRectify(K_l, D_l, K_r, D_r, imageSize, R_lr, T_lr, Rl, Rr, Pl, Pr, Q, cv::CALIB_FIX_INTRINSIC,
                  0, imageSize, &validROIL, &validROIR);
    cout << "finish rectify" << endl;
    //cout<<Pl<<endl;
    //cout<<Pr<<endl;
    // 相机校正
    cv::Mat M1l,M2l,M1r,M2r;
    //再采用映射变换计算函数initUndistortRectifyMap得出校准映射参数,该函数功能是计算畸变矫正和立体校正的映射变换
    cv::initUndistortRectifyMap(K_l,D_l,Rl,Pl.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,Rr,Pr.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    cout << "finish rectify" << endl;
    //保存详细标定参数，这里参照ORB-SLAM的参数文件格式
    string configYaml = root_result_path + "data_params_matlab.yaml";
    FileStorage storage(configYaml, FileStorage::WRITE);
    storage <<
            "Camera_fx", Pr.row(0).col(0).at<double>(),
            "Camera_fy", Pr.row(1).col(1).at<double>(),
            "Camera_cx", Pr.row(0).col(2).at<double>(),
            "Camera_cy", Pr.row(1).col(2).at<double>(),
            "Camera_k1", 0,
            "Camera_k2", 0,
            "Camera_p1", 0,
            "Camera_p2", 0,

            "Camera_width", 640,
            "Camera_height", 480,
            "Camera_fps", 50.0,
            "Camera_bf", (abs(Pr.row(0).col(3).at<double>()/1000.0)),
            "Camera_RGB", 1,
            "ThDepth", 50,

            "LEFT_height", size.height,
            "LEFT_width", size.width,
            "LEFT_D", D_l.t(),
            "LEFT_K", K_l,
            "LEFT_R", Rl,
            "LEFT_P", Pl,

            "RIGHT_height", size.height,
            "RIGHT_width", size.width,
            "RIGHT_D", D_r.t(),
            "RIGHT_K", K_r,
            "RIGHT_R", Rr,
            "RIGHT_P", Pr,

            "ORBextractor_nFeatures", 1200,
            "ORBextractor_scaleFactor", 1.2,
            "ORBextractor_nLevels", 8,
            "ORBextractor_iniThFAST", 20,
            "ORBextractor_minThFAST", 7,
            "Viewer_KeyFrameSize", 0.05,
            "Viewer_KeyFrameLineWidth", 1,
            "Viewer_GraphLineWidth", 0.9,
            "Viewer_PointSize", 2,
            "Viewer_CameraSize", 0.08,
            "Viewer_CameraLineWidth", 3,
            "Viewer_ViewpointX", 0,
            "Viewer_ViewpointY", -0.7,
            "Viewer_ViewpointZ", -1.8,
            "Viewer_ViewpointF", 500;
    storage.release();
    //检查标定效果
    CheckStereoCali(root_path, configYaml);
}