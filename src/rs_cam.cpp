#include "rs_cam.h"
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

RsCam::RsCam(int rgb_width, int rgb_height, int depth_width, int depth_height, int fps)
    : io_service(), serial_port(io_service, "/dev/ttyACM0")
{
    m_rgb_width = rgb_width;
    m_rgb_height = rgb_height;
    m_depth_width = depth_width;
    m_depth_height = depth_height;
    m_fps = fps;

    for (int i=0; i<8; i++)
        calibrationImages[i] = Mat(m_rgb_width, m_rgb_height, CV_8UC1);

    try {
        serial_port.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial_port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        serial_port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_port.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));
        msleep(1000);
    }
    catch(boost::system::system_error const& e)
    {
        cerr << "Startup lighting port connection, exception: " << e.what() << "\n";
    }


//    lightTimer = new QTimer(this);
//    lightTimer->setSingleShot(true);
//    connect(lightTimer, SIGNAL(timeout()), this, SLOT(lights_off()));
}

RsCam::~RsCam() {
}


void RsCam::lighting(int number, int intensity, int duration)
{
    serial_lock.lock();
    string command = "[ON W " + to_string(intensity) + " 1 " + to_string(number) + "]";
    try {
        boost::asio::write(serial_port, boost::asio::buffer(
                           command.c_str(), command.size()));
//        cout << "Command: " << command << endl;
    }
    catch(boost::system::system_error const& e)
    {
        cerr << "Command: " << command << ", exception: " << e.what() << endl;
    }

//    if (duration > -1)
//        lightTimer->start(duration);

    serial_lock.unlock();
}

void RsCam::lights_off()
{
    string command = "[OFF]";
    try {
        boost::asio::write(serial_port, boost::asio::buffer(
                           command.c_str(), command.size()));
    }
    catch(boost::system::system_error const& e)
    {
        cerr << "Command: " << command << ", exception: " << e.what() << endl;
    }
}

void RsCam::lights_on()
{
    string command = "[ALL_ON]";
    try {
        boost::asio::write(serial_port, boost::asio::buffer(
                           command.c_str(), command.size()));
    }
    catch(boost::system::system_error const& e)
    {
        cerr << "Command: " << command << ", exception: " << e.what() << endl;
    }
}

bool RsCam::open(int deviceIdx) {

    numCams = 1;


    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw runtime_error("No device detected. Is it plugged in?");



    if (numCams < 1) {
        cerr << "Camera not found or could not be opened." << endl;
        return false;
    }

    
    /* capture image with no LEDs to subtract ambient light */
    // captureAmbientImage();
    
    /* set average image intensity used by ps process for adjustment */
//    avgImgIntensity = mean(ambientImage)[0];

    return true;
}

void RsCam::start() {    
    m_cfg.enable_stream(RS2_STREAM_DEPTH, m_depth_width, m_depth_height, RS2_FORMAT_Z16, m_fps);
    m_cfg.enable_stream(RS2_STREAM_COLOR, m_rgb_width, m_rgb_height, RS2_FORMAT_RGB8, m_fps);

    m_profile = m_pipe.start(m_cfg);
    rs2::device selected_device = m_profile.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

//    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
//    {
//        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
//    }
//    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
//    {
//        // Query min and max values:
//        auto laser_power_range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
//        depth_sensor.set_option(RS2_OPTION_LASER_POWER, laser_power_range.max); // Set max power
//    }

    /* starting event loop, capturing fresh images */
//    eventLoopTimer = new QTimer();
//    eventLoopTimer->setSingleShot(true);
//    connect(eventLoopTimer, SIGNAL(timeout()), this, SLOT(captureFrame()));
//    connect(this, SIGNAL(stopped()), eventLoopTimer, SLOT(stop()));
//    eventLoopTimer->start(1000); ///m_fps);
    screenshotwithLight();
    //QTimer::singleShot(200, this, &RsCam::captureFrame);
}

void RsCam::stop() {
    m_pipe.stop();
    emit stopped();
}

void RsCam::setTestMode(bool toggle) {
    
    testMode = toggle;
}

bool RsCam::inTestMode() {
    
    return testMode;
}

int RsCam::avgImageIntensity() {
    
    return avgImgIntensity;
}

void RsCam::captureAmbientImage() {

    rs2::frameset frames ;

    msleep(250);
    lights_on();
    for (int i=0; i<5; i++) {
        frames = m_pipe.wait_for_frames();
        msleep(250);
    }

    vector<Vec3f> circles;
    double c_min_d = MAXFLOAT;
    int c_min_i = -1;
    double accuracy = 1.0;

    rs2::video_stream_profile rs_rgb_stream = m_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    while (c_min_d == MAXFLOAT) {
        frames = m_pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat gray = Mat(color.rows, color.cols, CV_8UC1);
        cvtColor(color, gray, COLOR_RGB2GRAY);
//        GaussianBlur(ambientImage, gray, Size(9, 9), 2, 2);
        Mat inv_gray = Mat(color.rows, color.cols, CV_8UC1);
        bitwise_not(gray, inv_gray);
        HoughCircles(inv_gray, circles, HOUGH_GRADIENT,
                         1,   // accumulator resolution (size of the image / 2)
                         20,  // minimum distance between two circles
                         100, // Canny high threshold
                         20, // minimum number of votes
                         0, 20); // min and max radius                         );
        Point frame_center(int(m_rgb_width*0.5), int(m_rgb_height*0.5));
        c_min_d = MAXFLOAT;
        c_min_i = -1;
        for( size_t i = 0; i < circles.size(); i++ )
        {
            auto c = circles[i];
            Point circle_center = Point(c[0], c[1]);
            int radius = c[2];

            double d = norm(frame_center - circle_center);
            if (d>50) continue;
            if (d < c_min_d) {
                c_min_d = d;
                c_min_i = i;
            }
            circle(color, circle_center, radius, Scalar(0,0,255), 1);
        }
        if (c_min_i!=-1) {
            auto c = circles[c_min_i];
            Point circle_center = Point(c[0], c[1]);
            int radius = c[2];
            circle(color, circle_center, radius, Scalar(255,0,0), 1);
        }
        
        Size patternsize(7,4); //interior number of corners
        vector<Point2f> corners; //this will be filled by the detected corners

        //CALIB_CB_FAST_CHECK saves a lot of time on images
        //that do not contain any chessboard corners
        Mat masked_image = gray.clone();
        

        std::vector<Mat> masked_images (0);
        while (true) {
            bool patternfound = findChessboardCorners(masked_image, patternsize, corners,
                                   CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE
                                   | CALIB_CB_FAST_CHECK);
            
            
            if (patternfound)
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                             TermCriteria(cv::TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
            else
                break;

            drawChessboardCorners(color, patternsize, Mat(corners), patternfound);

            if (corners.front().y > corners.back().y)
                std::reverse(corners.begin(), corners.end());
           
            Point vertices[4];
            vertices[0] = corners[0];
            vertices[1] = corners[(patternsize.height-1)*patternsize.width];
            vertices[2] = corners[(patternsize.height-1)*patternsize.width+(patternsize.width-1)];
            vertices[3] = corners[patternsize.width-1];
            cv::fillConvexPoly(masked_image, vertices, 4, Scalar(255,255,255));
            
            if (masked_images.size()!=3)
            {
                vertices[0] = corners[0];
                vertices[1] = corners[(patternsize.height-1)*patternsize.width];
                vertices[2] = corners[(patternsize.height-1)*patternsize.width+(patternsize.width-1)];
                vertices[3] = corners[patternsize.width-1];

                Mat masked_image(gray.size(), CV_8U, Scalar(0));
                cv::fillConvexPoly(masked_image, vertices, 4, Scalar(255,255,255));
                masked_images.push_back(masked_image);

                std::vector<Point3d> target_corner_points_world_3D;
                for (int i = 0; i < patternsize.height; i++)
                    for (int j = 0; j < patternsize.width; j++)
                        target_corner_points_world_3D.push_back(
                                    Point3d( i * patternsize.height, j * patternsize.width, 0 ));

                // Solve for pose of calibration target

                cv::Mat calibration_camera_matrix;
                cv::Mat calibration_distortion_coefficients;
                cv::Mat calibration_rotation_vector;
                cv::Mat calibration_translation_vector;

                auto rgb_video_frame = color_frame.as<rs2::video_frame>();
                const int rgb_width = rgb_video_frame.get_width();
                const int rgb_height = rgb_video_frame.get_height();
                rs2_intrinsics rs_rgb_instrinsics = rs_rgb_stream.get_intrinsics();

                calibration_camera_matrix = (Mat_<double>(3,3) << rs_rgb_instrinsics.fx, 0, rs_rgb_instrinsics.ppx, 0, rs_rgb_instrinsics.fy, rs_rgb_instrinsics.ppy, 0, 0, 1);
                calibration_distortion_coefficients = Mat::zeros(4, 1, DataType<double>::type);

                bool found_pose = solvePnP(target_corner_points_world_3D, corners, calibration_camera_matrix, calibration_distortion_coefficients, calibration_rotation_vector, calibration_translation_vector);
                if (found_pose) {
                    Mat R;
                    Rodrigues(calibration_rotation_vector, R); // Convert "rvec arg" m_calibration_rotation_vector to 3x3 rotation matrix R

                    // BUILD WORLD TO CAMERA TRANSFORMATION MATRIX
                    Mat w_M_c_ = Mat::eye(4, 4, R.type()); // M is 4x4
                    w_M_c_( Range(0,3), Range(0,3) ) = R * 1; // copies R into M
                    w_M_c_( Range(0,3), Range(3,4) ) = calibration_translation_vector * 1;

                    Mat inv_R = R.t();  // rotation of inverse

                    // CAMERA POSITION IN WORLD FRAME
                    Mat camera_position_in_world_3D = -inv_R * calibration_translation_vector;

                    // BUILD CAMERA TO WORLD TRANSFORMATION MATRIX
                    Mat inv_translation_vector = -inv_R * calibration_translation_vector; // translation of inverse
                    Mat c_M_w_ = Mat::eye(4, 4, R.type());
                    c_M_w_( Range(0,3), Range(0,3) ) = inv_R * 1; // copies R into T
                    c_M_w_( Range(0,3), Range(3,4) ) = inv_translation_vector * 1;


                }
            }

            if (masked_images.size()==3)
            {

                if (save_masked)
                {
                    cv::imshow("plane1", masked_images[0]);
                    cv::imshow("plane2", masked_images[1]);
                    cv::imshow("plane3", masked_images[2]);
                    cv::imwrite("plane1.png", masked_images[0]);
                    cv::imwrite("plane2.png", masked_images[1]);
                    cv::imwrite("plane3.png", masked_images[2]);
                    save_masked = false;
                }
            
            }
            
        }
        accuracy -= 0.1;
        emit newCamFrame(color.clone());
    }
    calibrationTarget = circles[c_min_i];

    lights_off();
    msleep(250);
    for (int i=0; i<5; i++) {
        frames = m_pipe.wait_for_frames();
        msleep(250);
    }

    frames = m_pipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    cvtColor(color, ambientImage, COLOR_RGB2GRAY);

}
/*
R = [0 0 1.0];
Nx     =   px - xc;
Ny     = -(py - yc);
Nz     = sqrt( radius^2 - Nx^2 - Ny^2 );
normal = [Nx, Ny, Nz];
normal = normal/radius;
NR     = normal(1)*R(1) + normal(2)*R(2) + normal(3)*R(3);
L(i,:) = 2 * Nz * normal - R;
*/

void RsCam::calibrate() {
    std::cout << "calibrate hit"<<std::endl;
    isCalibrating = true;
    captureAmbientImage();
    isCalibrating = false;
    QTimer::singleShot(20, this, &RsCam::captureFrame);
}

void RsCam::captureFrame() {
    
    std::cout<< "capture frame" <<std::endl;
    if (isCalibrating)
        return;

    currentLight++;
    if (currentLight==num_lights)
        currentLight = 0;

    if (!has_ambient || currentLight==num_lights) {
        std::cout<< "taking ambient" <<std::endl;
        captureAmbientImage();
        has_ambient = true;
        currentLight = 0;
    }

    Mat camFrame(m_rgb_height, m_rgb_width, CV_8UC1);
    imgIdx = (imgIdx+1) % 8;

    if (testMode) {
        camFrame = testImages[imgIdx].clone();
        /* faking camera image acquisition time */
        eventLoopTimer->setInterval(1000/FRAME_RATE);
    } else {
        double total_lights = 120;
        double mid_strip = 80;
        int light_id = int( (total_lights-mid_strip)/2 + currentLight*(mid_strip/8));
        
        lighting(light_id, 255);
        msleep(20);
        rs2::frameset frames = m_pipe.wait_for_frames();
        lighting(light_id, 0);
        rs2::frame color_frame = frames.get_color_frame();
        Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cvtColor(color, camFrame, COLOR_RGB2GRAY);

        camFrame -= ambientImage;
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Point center = Point(calibrationTarget[0], calibrationTarget[1]);
        Mat mask = Mat::zeros(color.rows, color.cols, CV_8UC1);
        int radius = calibrationTarget[2];
        int target_radius = int(radius*0.65);
        Rect target_roi(center.x-target_radius,center.y-target_radius,target_radius*2,target_radius*2);
        rectangle(color, target_roi, Scalar(255,0,0));
        mask(target_roi) = 255;
        minMaxLoc( camFrame, &minVal, &maxVal, &minLoc, &maxLoc, mask );
        circle(color, maxLoc, 2, Scalar(255,0,0), 1);

        emit newCamFrame(color.clone());

        calibrationImages[currentLight] = camFrame;

    }
    
//    msleep(1000);
//    eventLoopTimer->start(500);
//    eventLoopTimer = QTimer::singleShot(200, this, &RsCam::captureFrame);
    // emit newCamFrame(ambientImage.clone());
    
    /* remove ambient light */
//     camFrame -= ambientImage;
//     emit newCamFrame(camFrame.clone());

    /* assigning image id (current active LED) to pixel in 0,0 */
    // camFrame.at<uchar>(0, 0) = imgIdx;
    
    QTimer::singleShot(20, this, &RsCam::captureFrame);
    // emit camFrame;
}

void RsCam::msleep(unsigned long msecs) {

    QTime sleepTime = QTime::currentTime().addMSecs(msecs);
    while (QTime::currentTime() < sleepTime) {
        QCoreApplication::processEvents(QEventLoop::AllEvents);
    }
}
void RsCam::screenshotwithLight() {
    rs2::frameset frames;
    bool saved_ambient = false;
    for (int i=0; i<5; i++) 
    {
        frames = m_pipe.wait_for_frames();
        msleep(250);
    }
    
    while (true) {
       

        frames = m_pipe.wait_for_frames();
        msleep(10);
        rs2::frame color_frame = frames.get_color_frame();
        Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat gray = Mat(color.rows, color.cols, CV_8UC1);
        cvtColor(color, gray, COLOR_RGB2GRAY);

        if (saved_ambient == false)
        {
            imwrite("ambient.png", gray);
            saved_ambient = true;

        }
        if (saveimage)
        {
            
            std::map <int, Mat> light_map;
            for (int i = 0; i < num_lights; i++)
            {
                //msleep(250);
                lighting(i, 255);
                msleep(10);
                frames = m_pipe.wait_for_frames();
                color_frame = frames.get_color_frame();
                Mat current_light_color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
                Mat current_light_gray = Mat(color.rows, color.cols, CV_8UC1);
                cvtColor(current_light_color, current_light_gray, COLOR_RGB2GRAY);
                light_map[i] = current_light_color;
                std::string filename = "cap_test_light"+ std::to_string(i) + ".png";
                imwrite(filename, current_light_gray);
                std::cout << "saving in :  " << filename <<std::endl;
                Mat diff = gray - current_light_gray;
                filename = "cap_test_light_diff"+ std::to_string(i) + ".png";
                imwrite(filename, diff);
                lights_off();
//                msleep(10);

            }
            saveimage = false;
        }

        emit newCamFrame(color.clone());
    }
}
void RsCam::save_image()
{
    saveimage = true;   
}
void RsCam::save_mask()
{
    save_masked = true;   
}

void RsCam::sendlight()
{
    std::cout<< "Lighting" <<std::endl;
    lights_off();
    currentLight = currentLight + 1;
    if (currentLight==num_lights)
        std::cout<< "all lights are activate" <<std::endl;
        currentLight = 0;

    lighting(currentLight, 255);
   
}
