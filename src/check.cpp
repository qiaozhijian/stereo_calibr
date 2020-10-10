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

using namespace cv;
using namespace std;


int main(int argc, char *argv[]) {
    cv::CommandLineParser parser(argc, argv,
                                 "{w|11|}{h|8|}{s|15|}{d|/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round2/cali|}{show|true|}{help||}");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    string root_path = parser.get<string>("d");
    string root_result_path = root_path + "/result/";
    createDirectory(root_result_path);
    string configYaml = root_result_path + "data_params_matlab.yaml";

     CheckStereoCali(root_path, configYaml);
}