//
// Created by shenyl on 2020/6/26.
// function：读取标定结果文件并进行图像的双目矫正、匹配和深度估计，生成校正后图像和视差图
// usage：./stereo_match -d=<dir default=/home/shenyl/Documents/sweeper/data/> -show=<if_show default=True>
//

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <map>
#include <string>

using namespace cv;
using namespace std;

static int print_help()
{
    cout <<
         " Given the root dir of the exp dir and the match algorithm \n"
         " output the rectified images and sparities\n"<< endl;
    cout << "Usage:\n // usage：./stereo_calib -w=<board_width default=11> -h=<board_height default=8> -s=<square_size default=15> -d=<dir default=/home/shenyl/Documents/sweeper/data/> -show=<if_show default=False>\n" << endl;
}


//attention: data must be read as double to be the same with the parameter definition of cv::stereoRectify function
void read_calib_parameters(string calib_parameters, Mat& cameraMatrix_L, Mat& distCoeffs_L, Mat& cameraMatrix_R, Mat& distCoeffs_R,
        Mat& R, Mat& T, Size& imageSize)
{

    ifstream calib_file(calib_parameters);
    if (!calib_file)
    {
        cout<<calib_parameters<<"is not exist"<<endl;
    }
    string line;
    enum StringValue { evNotDefined,
        evStringValue1,
        evStringValue2,
        evStringValue3,
        evStringValue4,
        evStringValue5,
        evStringValue6,
        evStringValue7,
        evEnd };
    map<std::string, StringValue> s_mapStringValues;
    s_mapStringValues["image size"] = evStringValue1;
    s_mapStringValues["cameraMatrix_L"] = evStringValue2;
    s_mapStringValues["cameraMatrix_R"] = evStringValue3;
    s_mapStringValues["distCoeffs_L"] = evStringValue4;
    s_mapStringValues["distCoeffs_R"] = evStringValue5;
    s_mapStringValues["R"] = evStringValue6;
    s_mapStringValues["T"] = evStringValue7;
    s_mapStringValues["end"] = evEnd;

    while (getline(calib_file, line))
    {
        switch(s_mapStringValues[line]){
            case evStringValue1:  //image size
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ' ');
                double w = atof(line.c_str());
                getline(calib_file, line, ' ');
                getline(calib_file, line, ']');
                double h = atof(line.c_str());
                imageSize = Size(w, h);;
//                cout<< "imagesize"<< endl<< imageSize<<endl;
                break;
            }
            case evStringValue2:  //cameraMatrix_L
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double m11 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m12 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m13 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m21 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m22 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m23 = atof(line.c_str());
                cameraMatrix_L = (Mat_<double>(3, 3) << m11, m12, m13, m21, m22, m23, 0, 0, 1);
//                cout<< "cameraMatrix_L"<< endl<< cameraMatrix_L<<endl;
                getline(calib_file, line);
                break;
            }
            case evStringValue3: //cameraMatrix_R
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double m11 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m12 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m13 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m21 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m22 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m23 = atof(line.c_str());
                cameraMatrix_R = (Mat_<double>(3, 3) << m11, m12, m13, m21, m22, m23, 0, 0, 1);
//                cout<< "cameraMatrix_R"<< endl<< cameraMatrix_R<<endl;
                getline(calib_file, line);
                break;
            }
            case evStringValue4: //distCoeffs_L
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double v1 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v2 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v3 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v4 = atof(line.c_str());
                getline(calib_file, line, ']');
                double v5 = atof(line.c_str());
                distCoeffs_L = (Mat_<double>(1, 5) << v1, v2, v3, v4, v5);
//                cout<< "distCoeffs_L"<< endl<< distCoeffs_L<<endl;
                break;
            }
            case evStringValue5: //distCoeffs_R
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double v1 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v2 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v3 = atof(line.c_str());
                getline(calib_file, line, ',');
                double v4 = atof(line.c_str());
                getline(calib_file, line, ']');
                double v5 = atof(line.c_str());
                distCoeffs_R = (Mat_<double>(1, 5) << v1, v2, v3, v4, v5);
//                cout<< "distCoeffs_R"<< endl<< distCoeffs_R<<endl;
                break;
            }
            case evStringValue6: //R
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ',');
                double m11 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m12 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m13 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m21 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m22 = atof(line.c_str());
                getline(calib_file, line, ';');
                double m23 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m31 = atof(line.c_str());
                getline(calib_file, line, ',');
                double m32 = atof(line.c_str());
                getline(calib_file, line, ']');
                double m33 = atof(line.c_str());
                R = (Mat_<double>(3, 3) << m11, m12, m13, m21, m22, m23, m31, m32, m33);
//                cout<< "R"<< endl<< R<<endl;
                break;
            }
            case evStringValue7: //T
            {
                getline(calib_file, line, '[');
                getline(calib_file, line, ';');
                double v1 = atof(line.c_str());
                getline(calib_file, line, ';');
                double v2 = atof(line.c_str());
                getline(calib_file, line, ']');
                double v3 = atof(line.c_str());
                T = (Mat_<double>(3, 1) << v1, v2, v3);
//                cout<< "T"<< endl<< T<<endl;
                break;
            }
        }

    }
}

Rect stereoRectification(Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2,
                         Size& imageSize, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2)
{
    Rect validRoi[2];
    cout<<cameraMatrix1<<endl<<distCoeffs1<<endl<<cameraMatrix2<<endl<<distCoeffs2<<endl<<imageSize<<endl<<R <<endl<<T<<endl;
    stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize,
                  R, T, R1, R2, P1, P2, Q, 0, -1, imageSize, &validRoi[0], &validRoi[1]);
    cout << "R1:" << endl;
    cout << R1 << endl;
    cout << "R2:" << endl;
    cout << R2 << endl;
    cout << "P1:" << endl;
    cout << P1 << endl;
    cout << "P2:" << endl;
    cout << P2 << endl;
    cout << "Q:" << endl;
    cout << Q << endl;
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize, CV_32FC1, mapl1, mapl2);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize, CV_32FC1, mapr1, mapr2);
    return validRoi[0], validRoi[1];
}

bool Rectification(string root_path, string imageName_L, string imageName_R, Mat& img1_rectified,
                               Mat& img2_rectified, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2, Rect validRoi[2], int count) {
    Size imageSize;
    string left_rectified = root_path + "rectified/left/";
    string right_rectified = root_path + "rectified/right/";
    string pairs_rectified = root_path + "rectified/pairs/";
    Mat img1 = imread(imageName_L);
    Mat img2 = imread(imageName_R);
    if (img1.empty() | img2.empty()) {
        cout << "图像为空" << endl;
    }
    Mat gray_img1, gray_img2;
    cvtColor(img1, gray_img1, COLOR_BGR2GRAY);
    cvtColor(img2, gray_img2, COLOR_BGR2GRAY);
    imageSize.width = img1.cols; // 获取图片的宽度
    imageSize.height = img1.rows; // 获取图片的高度
    Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC1); // 注意数据类型
    Mat canLeft = canvas(Rect(0, 0, imageSize.width, imageSize.height));
    Mat canRight = canvas(Rect(imageSize.width, 0, imageSize.width, imageSize.height));
    gray_img1.copyTo(canLeft);
    gray_img2.copyTo(canRight);

    remap(gray_img1, img1_rectified, mapl1, mapl2, INTER_LINEAR);
    remap(gray_img2, img2_rectified, mapr1, mapr2, INTER_LINEAR);

    char left_file[200];
    sprintf(left_file, "%06d.jpg", count);
    imwrite(left_rectified + left_file, img1_rectified);
    char right_file[200];
    sprintf(right_file, "%06d.jpg", count);
    imwrite(right_rectified + right_file, img2_rectified);

    img1_rectified.copyTo(canLeft);
    img2_rectified.copyTo(canRight);

    rectangle(canLeft, validRoi[0], Scalar(255, 255, 255), 5, 8);
    rectangle(canRight, validRoi[1], Scalar(255, 255, 255), 5, 8);
    for (int j = 0; j <= canvas.rows; j += 16)
        line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

    char pairs_file[200];
    sprintf(pairs_file, "%06d.jpg", count);
    imwrite(pairs_rectified + pairs_file, canvas);
//        imshow("rectified", canvas);
//        if (waitKey(0) == 27) {
//            destroyAllWindows();
//        }
}

//todo:compute disparity image with rectified image
bool computeDisparityImage(Mat& img1_rectified,
                           Mat& img2_rectified, Mat& mapl1, Mat& mapl2, Mat& mapr1, Mat& mapr2, Rect validRoi[2], Mat& disparity, int count)
{
    // 进行立体匹配 bm
    Mat disparity_bm;
    Ptr<StereoBM> bm;
    bm = StereoBM::create(16, 9); // Ptr<>是一个智能指针
    bm->compute(img1_rectified, img2_rectified, disparity_bm); // 计算视差图
    disparity_bm.convertTo(disparity_bm, CV_32F, 1.0 / 16);
    // 归一化视差映射
    normalize(disparity_bm, disparity_bm, 0, 256, NORM_MINMAX, -1);


    // 进行立体匹配 sgbm
    Mat disparity_sgbm;
    Ptr<StereoSGBM> sgbm;
;
//    }
    cout<<"image type: "<<img1_rectified.type()<<endl;
    cout<<"image shape"<<img1_rectified.cols << "," << img1_rectified.rows<<endl;
    sgbm = cv::StereoSGBM::create(
            0, 256, 16, 4*8*8*8, 4*32*8*8, 1, 1, 10, 200, 200, cv::StereoSGBM::MODE_SGBM);
    sgbm->compute(img1_rectified, img2_rectified, disparity_sgbm); // 计算视差图

    disparity_sgbm.convertTo(disparity_sgbm, CV_32F, 1.0 / 16);
    // 归一化视差映射
    normalize(disparity_sgbm, disparity_sgbm, 0, 256, NORM_MINMAX, CV_8U);
    cv::imwrite("../data/disparity_bm.jpg", disparity_bm);
    cv::imwrite("../data/disparity_sbgm.jpg", disparity_sgbm);


    disparity = disparity_sgbm;
    return true;
}

int main(int argc,char *argv[])
{
    cv::CommandLineParser parser(argc, argv, "{d|/home/shenyl/Documents/sweeper/data/|}{help||}");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    String root_path = parser.get<String>("d");
    String imageList_L = root_path + "img/test_file_left.txt";
    String imageList_R = root_path + "img/test_file_right.txt";
    String calib_parameters =  root_path + "calib_img/stereocalibrateresult_L.txt";
    String rectified_parameters = root_path + "calib_img/stereoRectifyParams.txt";

    Mat cameraMatrix_L = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 相机的内参数
    Mat cameraMatrix_R = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 初始化相机的内参数
    Mat distCoeffs_L = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 相机的畸变系数
    Mat distCoeffs_R = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 初始化相机的畸变系数
    Mat R, T;
    Size imageSize;
    Rect validRoi[2];//双目矫正有效区域
    Mat R1, R2, P1, P2, Q; // 立体校正参数
    Mat mapl1, mapl2, mapr1, mapr2; // 图像重投影映射表
    Mat img1_rectified, img2_rectified; // 校正图像
    Mat disparity; // 视差图


    //Step1 read camera parameters
    read_calib_parameters(calib_parameters, cameraMatrix_L, distCoeffs_L, cameraMatrix_R, distCoeffs_R,
            R, T, imageSize);
//    cout<< R << endl<< T << endl << cameraMatrix_L << endl << distCoeffs_L << endl << cameraMatrix_R << endl<< distCoeffs_R<<endl;
    //Step2 get stereo rectified parameters
    validRoi[0], validRoi[1] = stereoRectification(cameraMatrix_L, distCoeffs_L, cameraMatrix_R, distCoeffs_R,
                                                   imageSize, R, T, R1, R2, P1, P2, Q, mapl1, mapl2, mapr1, mapr2);

    string left_rectified = root_path + "rectified/left/";
    string right_rectified = root_path + "rectified/right/";
    string pairs_rectified = root_path + "rectified/pairs/";
    string command = "mkdir -p " + left_rectified;
    system(command.c_str());
    command = "mkdir -p " + right_rectified;
    system(command.c_str());
    command = "mkdir -p " + pairs_rectified;
    system(command.c_str());

    ifstream imageStore_L(imageList_L); // 打开存放标定图片名称的txt
    ifstream imageStore_R(imageList_R); // 打开存放标定图片名称的txt
    string imageName_L; // 读取的标定图片的名称
    string imageName_R; // 读取的标定图片的名称
    int count = 0 ;
    // for each pair rectified images and generate sparity
    while (getline(imageStore_L, imageName_L))
    {
        getline(imageStore_R, imageName_R);
        // Step3 rectified images
        Rectification(root_path, imageName_L, imageName_R, img1_rectified, img2_rectified, mapl1, mapl2, mapr1, mapr2, validRoi, count++);
        // Step4 generate sparities
        computeDisparityImage(root_path, img1_rectified,
                img2_rectified, disparity, count, algortirhm);
    }




    //Step3 estimation 3D positions




}

