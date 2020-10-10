//
// Created by qzj on 2020/6/24.
//
#include "util.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include "util.h"
#include <stdio.h>
using namespace std;
using namespace cv;

string getDirEnd(string dataset_dir)
{
    string end;
    unsigned int iSize = dataset_dir.size();
    unsigned int i = 0;
    for(i = 0; i < iSize; i++)
    {
        if(dataset_dir.at(i)=='/' && i!=iSize-1)
            end=dataset_dir.substr(i+1);
    }
    if (end[end.size()-1]=='/')
        end.pop_back();
    return end;
}

string removeExtension(string filewhole)
{
    replace_str(filewhole,"..","**");
    //1.获取不带路径的文件名
    string::size_type iPos = filewhole.find_last_of('\\') + 1;
    string filename = filewhole.substr(iPos, filewhole.length() - iPos);
    //**/**/dataset/room216/01/left/005466.jpg
    //cout << filename << endl;

    //2.获取不带后缀的文件名
    string name = filename.substr(0, filename.rfind("."));
    //**/**/dataset/room216/01/left/005466
    //cout << name << endl;

    //3.获取后缀名
    string suffix_str = filename.substr(filename.find_last_of('.') + 1);
    //jpg
    //cout << suffix_str << endl;
    name = getDirEnd(name);
    return name;
}

int sting2Int(string s)
{
    stringstream ss;
    int x;
    ss<<s;
    ss>>x;
    return x;
}

void getSortedImages(const boost::filesystem::path &img_dir, function<bool(const string &)> filter,
                     function<bool(const string &, const string &)> comparator, vector<string> &img_paths) {

    // get a sorted list of files in the img directories
    if (!boost::filesystem::exists(img_dir) ||
        !boost::filesystem::is_directory(img_dir))
        throw runtime_error("[Dataset] Invalid images subfolder");

    // get all files in the img directories
    list<string> all_imgs;
    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(img_dir), {})) {
        boost::filesystem::path filename_path = entry.path().filename();
        if (boost::filesystem::is_regular_file(entry.status()) &&
            (filename_path.extension() == ".png"  ||
             filename_path.extension() == ".jpg"  ||
             filename_path.extension() == ".jpeg" ||
             filename_path.extension() == ".pnm"  ||
             filename_path.extension() == ".tiff")) {
            all_imgs.push_back(filename_path.string());
        }
    }

    // sort
    img_paths.clear();
    img_paths.reserve(all_imgs.size());
    for (const string &filename : all_imgs)
        if (!filter(filename)) img_paths.push_back(filename);

    if (img_paths.empty())
        //cout<<"[Dataset] Invalid image names?"<<endl;
        throw runtime_error("[Dataset] Invalid image names?");

    sort(img_paths.begin(), img_paths.end(), comparator);

    for (string &filename : img_paths)
        filename = (img_dir / filename).string();
}

void getStereoSortedImages(const string root_path,vector<std::string> &img_l_paths, vector<std::string> &img_r_paths)
{
    boost::filesystem::path dataset_base(root_path);
    if (!boost::filesystem::exists(dataset_base) ||
        !boost::filesystem::is_directory(dataset_base))
        throw std::runtime_error("[Dataset] Invalid directory");

    boost::filesystem::path img_l_dir = dataset_base / "left";
    boost::filesystem::path img_r_dir = dataset_base / "right";

    boost::regex expression("^[^0-9]*([0-9]+\\.?+[0-9]*)[^0-9]*\\.[a-z]{3,4}$");
    boost::cmatch what;
    auto filename_filter = [&expression, &what](const std::string &s) {
        return !boost::regex_match(s.c_str(), what, expression);
    };

    auto sort_by_number = [&expression, &what](const std::string &a, const std::string &b) {
        double n1, n2;

        if (boost::regex_match(a.c_str(), what, expression))
            n1 = std::stod(what[1]);
        else
            throw std::runtime_error("[Dataset] Unexpected behaviour while sorting filenames");

        if (boost::regex_match(b.c_str(), what, expression))
            n2 = std::stod(what[1]);
        else
            throw std::runtime_error("[Dataset] Unexpected behaviour while sorting filenames");

        return (n1 < n2);
    };

    getSortedImages(img_l_dir, filename_filter, sort_by_number, img_l_paths);
    getSortedImages(img_r_dir, filename_filter, sort_by_number, img_r_paths);
}


void copy_file(std::string src_, std::string dst_)
{
    char* src = (char*)src_.c_str();
    char* dst = (char*)dst_.c_str();
    using namespace std;
    ifstream in(src,ios::binary);
    ofstream out(dst,ios::binary);
    if (!in.is_open()) {
        cout << "error open file " << src << endl;
        exit(EXIT_FAILURE);
    }
    if (!out.is_open()) {
        cout << "error open file " << dst << endl;
        exit(EXIT_FAILURE);
    }
    if (src == dst) {
        cout << "the src file can't be same with dst file" << endl;
        exit(EXIT_FAILURE);
    }
    char buf[2048];
    long long totalBytes = 0;
    while(in)
    {
        //read从in流中读取2048字节，放入buf数组中，同时文件指针向后移动2048字节
        //若不足2048字节遇到文件结尾，则以实际提取字节读取。
        in.read(buf, 2048);
        //gcount()用来提取读取的字节数，write将buf中的内容写入out流。
        out.write(buf, in.gcount());
        totalBytes += in.gcount();
    }
    in.close();
    out.close();
}
// 从左到右依次判断文件夹是否存在,不存在就创建
// example: /home/root/mkdir/1/2/3/4/
// 注意:最后一个如果是文件夹的话,需要加上 '\' 或者 '/'
int32_t createDirectory(const std::string &directoryPath)
{
    uint32_t dirPathLen = directoryPath.length();
    if (dirPathLen > MAX_PATH_LEN)
    {
        return 1;
    }
    char tmpDirPath[MAX_PATH_LEN] = { 0 };
    for (uint32_t i = 0; i < dirPathLen; ++i)
    {
        tmpDirPath[i] = directoryPath[i];
        if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/')
        {
            if (ACCESS(tmpDirPath, 0) != 0)
            {
                int32_t ret = MKDIR(tmpDirPath);
                if (ret != 0)
                {
                    return ret;
                }
            }
        }
    }
    return 0;
}


/**
 * 字符串替换函数
 * #function name   : replace_str()
 * #param str       : 操作之前的字符串
 * #param before    : 将要被替换的字符串
 * #param after     : 替换目标字符串
 * #return          : void
 */
void replace_str(std::string& str, const std::string& before, const std::string& after)
{
    for (std::string::size_type pos(0); pos != std::string::npos; pos += after.length())
    {
        pos = str.find(before, pos);
        if (pos != std::string::npos)
            str.replace(pos, before.length(), after);
        else
            break;
    }
}



void  CheckStereoCali(string root_path, string configYaml)
{
    FileStorage fsSettings(configYaml, FileStorage::READ);
    cout<<configYaml<<endl;
    //FileStorage fsSettings(root_result_path + "data_params.yaml", FileStorage::READ);

    vector<std::string> img_l_paths, img_r_paths;
    Mat img1_rectified, img2_rectified; // 校正图像
    getStereoSortedImages(root_path, img_l_paths, img_r_paths);

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT_K"] >> K_l;
    fsSettings["RIGHT_K"] >> K_r;

    fsSettings["LEFT_P"] >> P_l;
    fsSettings["RIGHT_P"] >> P_r;

    fsSettings["LEFT_R"] >> R_l;
    fsSettings["RIGHT_R"] >> R_r;

    fsSettings["LEFT_D"] >> D_l;
    fsSettings["RIGHT_D"] >> D_r;

    int rows_l = fsSettings["LEFT_height"];
    int cols_l = fsSettings["LEFT_width"];
    int rows_r = fsSettings["RIGHT_height"];
    int cols_r = fsSettings["RIGHT_width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
       rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return ;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    vector<string>::iterator iter_l, iter_r;
    for (iter_l = img_l_paths.begin(), iter_r = img_r_paths.begin();
         iter_l != img_l_paths.end(); iter_l++, iter_r++) // 读取txt的每一行（每一行存放了一张标定图片的名称）
    {
        // Read left and right images from file
        imLeft = cv::imread(*iter_l, CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(*iter_r, CV_LOAD_IMAGE_UNCHANGED);

        if (imLeft.empty()) {
            cerr << endl << "Failed to load image at: "
                 << *iter_l << endl;
            return ;
        }
        if (imRight.empty()) {
            cerr << endl << "Failed to load image at: "
                 << *iter_r << endl;
            return ;
        }

        //cv::imshow("imRight",imRight);
//         校正
        cv::remap(imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);

        cv::Size imageSize(cols_l,rows_l);
        cv::Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC3);
        cv::Mat canLeft = canvas(cv::Rect(0, 0, imageSize.width, imageSize.height));
        cv::Mat canRight = canvas(cv::Rect(imageSize.width, 0, imageSize.width, imageSize.height));
        //cout<<"canLeft: "<<imLeft.type()<<" canvas: "<<canvas.type()<<endl;
        imLeftRect(cv::Rect(0, 0, imageSize.width, imageSize.height)).copyTo(canLeft);
        imRightRect(cv::Rect(0, 0, imageSize.width, imageSize.height)).copyTo(canRight);
        //cout << "done" << endl;
        for (int j = 0; j <= canvas.rows; j += 16)
            cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
        //cout << "stereo rectify done" << endl;
        cv::imshow("canvas",canvas);
        cv::waitKey(0);
    }
}