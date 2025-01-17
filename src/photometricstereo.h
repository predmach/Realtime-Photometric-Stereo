#ifndef PHOTOMETRIC_STEREO_H
#define PHOTOMETRIC_STEREO_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <sys/time.h>

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "OpenCL/cl.hpp"
#include "oclutils.h"
#include "config.h"

class PhotometricStereo : public QObject {

    Q_OBJECT
    
public:
    PhotometricStereo(int width, int height, int imageIntensity);
    ~PhotometricStereo();
    void execute();
    float getMaxPQ();
    float getLambda();
    float getMu();
    float getMinIntensity();
    float getUnsharpScale();
    
public slots:
    void setImage(cv::Mat image);
    void setMaxPQ(double val);
    void setLambda(double val);
    void setMu(double val);
    void setMinIntensity(int val);
    void setUnsharpScale(int val);
    
signals:
    void executionTime(QString timeMillis);
    void modelFinished(std::vector<cv::Mat> MatXYZN);
    
private:
    /* device variables */
    std::vector<cl::Device> devices;
    cl::Program program;
    cl::Context context;
    cl::CommandQueue queue;
    cl::Kernel calcNormKernel, integKernel, updateNormKernel;
    
    /* opencl buffer */
    cl::Image2D cl_img1, cl_img2, cl_img3, cl_img4, cl_img5, cl_img6, cl_img7, cl_img8;
    cl::Buffer cl_Pgrads, cl_Qgrads;
    cl::Buffer cl_Sinv, cl_N;
    cl::Buffer cl_P, cl_Q, cl_Z;
    
    /* debugging variables */
    cl_int error;
    cl::Event event;
    QFuture<void> future;
    QMutex mutex;
    
    /* ps parameters adjustable by user input */
    float maxpq;
    float lambda, mu;
    int minIntensity;
    float unsharpScaleFactor;
    
    /* ps images */
    std::vector<cv::Mat> psImages;
    int imgIdx;
    
    /* model size */
    int width, height;
    
    /* non-changing x,y coordinates of 3d model */
    cv::Mat XCoords, YCoords;
    
    /* light directions */
    cv::Mat lightSrcsInv;
    
    cv::Mat getGlobalHeights(cv::Mat Pgrads, cv::Mat Qgrads);
    long getMilliSecs();
    cv::Mat readCalibratedLights();
};


#endif