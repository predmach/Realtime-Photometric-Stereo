#ifndef RS_CAM_H
#define RS_CAM_H

#define STROBE_0_CNT    0x1500
#define STROBE_1_CNT    0x1504
#define STROBE_2_CNT    0x1508
#define STROBE_3_CNT    0x150C
#define STROBE_CTRL_INQ 0x1300
#define STROBE_0_INQ    0x1400
#define STROBE_1_INQ    0x1404
#define STROBE_2_INQ    0x1408
#define STROBE_3_INQ    0x140C
#define PIO_DIRECTION   0x11F8
#define INITIALIZE      0x000

#include <iostream>
#include <vector>
#include <string>
#include <mutex>

#include <boost/asio.hpp>
//#include "boost/asio.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QObject>
#include <QTimer>
#include <QtCore/QTime>
#include <QCoreApplication>
#include <QFileDialog>
#include <librealsense2/rs.hpp>
#include "calibration.h"
#include "photometricstereo.h"
//#include "lights.h"

class RsCam : public QObject {
    Q_OBJECT

public:
    RsCam(int rgb_width=640, int rgb_height=480, int depth_width=640, int depth_height=480, int fps=15);
    ~RsCam();
    bool open(int deviceIdx);
    void stop();
    void reset() {};
    void setTestMode(bool toggle);
    int avgImageIntensity();
    bool inTestMode();

    void lighting(int number, int intensity, int duration=-1);
    void set_width_height();
    int m_rgb_width;
    int m_rgb_height;
    int m_depth_width;
    int m_depth_height;
    int model_cropped_width;
    int model_cropped_height;
    int m_fps;
    int lighid = 0;
    bool save_masked = false;
    bool get_plane_normal = false;
    double calibration_pattern_scale_{0.017}; //m
    PhotometricStereo *ps;
    bool save_screenshot = true;
public slots:
    void start();
    void calibrate();
    void sendlight();
    void captureFrame();
    //void captureFrame_new();
    void save_image();
    void save_mask();
    void test_get_normal();
private slots:
    
    void lights_off();
    void lights_on();
    
    void screenshotwithLight();
    // void save_image();
    

signals:
    void newCamFrame(cv::Mat frame);
    void newCroppedFrame(cv::Mat frame);
    void newFrames(std::map<int, cv::Mat>);
    void newScale(int, int, int);
    void stopped();
    void stop_frame_timer();

private:
    boost::asio::io_service io_service;
    boost::asio::serial_port serial_port;
    std::mutex serial_lock;

    bool isCalibrating=false;
    bool saveimage = false;
    cv::Vec3i calibrationTarget;
    cv::Mat calibrationImages[8];
    std::map < int, cv::Mat> captured_images;
    std::map <int, cv::Mat> captured_diff;
    QTimer *lightTimer;
//    Lights *lights;
    int currentLight=0;
    int num_lights=105; //77;

    int numCams;

    int light_id = 13;
    int id = 0;
    std::vector<int> light_idx {20, 60, 70, 80, 120, 140, 160, 180};
    rs2::config m_cfg; // RealSense config
    rs2::pipeline m_pipe; // Stream processing
    rs2::pipeline_profile m_profile;
    rs2::frameset m_frames; // frames from pipeline

//    rs2::frame_queue m_original_data;
    rs2::frame_queue m_filtered_depth_data;
    rs2::frame_queue m_filtered_rgb_data;


    QTimer *eventLoopTimer;
    std::vector<cv::Mat> testImages;
    cv::Mat ambientImage;
    bool has_ambient=false;

    // Contruct a pipeline which abstracts the device
//    rs2::pipeline* pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    // rs2::config* cfg;

    bool testMode=false;
    int imgIdx;
    int avgImgIntensity;
    int FRAME_RATE;
//    int camFrameWidth, camFrameHeight;
    
    void captureAmbientImage();
    void captureAmbientImage_new();
    void capture_calibration_images();
    /* Own implementation of sleep, processing all qt events while sleeping/waiting */
    void msleep(unsigned long msecs);
    cv::Rect cropped_rect;
   
    static void on_MouseHandle(int event, int x, int y, int flags, void* param);
    static void DrawRectangle(cv::Mat& img, cv::Rect box);
    
    bool cropped_need = true;
    
};

#endif
