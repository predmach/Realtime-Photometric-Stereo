#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <iostream>
#include <stdio.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "config.h"
#include <fstream>

class Calibration {
  
public:
    static void withFourPlanes();
    static void withThreePlane(cv::Mat normals);
    static void get_plane_normals(cv::Mat normals);
    bool saveMatBinary(const std::string& filename, const cv::Mat& mat);
    bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);

};

#endif