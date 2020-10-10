#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;
using namespace cv;

int main()
{
    string strSettingsFile = "./setting.yaml";
    //1.创建文件
    cv::FileStorage fwrite(strSettingsFile,cv::FileStorage::WRITE);
    //2.写入数据
    float fx = 100.0;
    float fy = 101.0;
    cv::Mat sample = (cv::Mat_<double>(3, 3) << 0, -1, 0, -1, 0, 0, 0, 0, 1);
    fwrite<<"fx "<< fx;
    fwrite<<"fy "<< fy;
    fwrite<<"sample "<< sample;
    //3.关闭文件
    fwrite.release();

    string strwriting = "/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round2/cali/result/data_params_matlab.yaml";
    //1.读取文件指针
    cv::FileStorage fread(strwriting.c_str(),cv::FileStorage::READ);

    //2.判断是否打开成功
    if(!fread.isOpened())
    {
        cout<<"Failed to open settings file at: "<<strwriting<<endl;
        return 0 ;
    }
    else cout<<"success to open file at: "<<strwriting<<endl;

    //3.打开文件后读取数据
    float fxread,fyread;
    cv::Mat sample_read;
    //fread["fx"]>>fxread;
    //fread["fy"]>>fyread;
    fread["LEFT_K"]>>sample_read;
    //cout<<"fxread="<<fxread<<endl;
    //cout<<"fyread="<<fxread<<endl;
    cout<<"LEFT_K="<<sample_read<<endl;

    //4.关闭文件
    fread.release();

    return 0;
}

