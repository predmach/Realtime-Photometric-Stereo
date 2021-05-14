#include "rs_cam.h"
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;
#define WINDOW_NAME "Get ROI"
bool g_bDrawingBox = false;
cv::Rect g_rectangle;
bool complete_rect = true;


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

void RsCam::set_width_height()
{
    rs2::frameset frames;
    for (int i=0; i<5; i++) 
    {
        frames = m_pipe.wait_for_frames();
        msleep(250);
    }
    rs2::frame color_frame = frames.get_color_frame();
    Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    Mat grey;
    cvtColor(color, grey, COLOR_RGB2GRAY);
    namedWindow(WINDOW_NAME);
	Mat tempImage;
    setMouseCallback(WINDOW_NAME, on_MouseHandle, (void*) &grey);
        
	while (1) {
        color.copyTo(tempImage);
            
        if (g_bDrawingBox)
            DrawRectangle(tempImage, g_rectangle);
        imshow(WINDOW_NAME, tempImage);
            //char c = getchar();

        if (complete_rect==false)  // stop drawing rectanglge if the key is 'ESC'
            break;

	}
    cropped_rect = g_rectangle;
    if (!complete_rect)
    {
        cv::Mat croppedImage = color(cropped_rect);
        cv::imshow("cropped", croppedImage);
        cv::destroyAllWindows();
    }
    
    model_cropped_width = g_rectangle.width;
    model_cropped_height = g_rectangle.height;
    emit newScale(model_cropped_width, model_cropped_height, avgImageIntensity());

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
    //screenshotwithLight();
    //captureAmbientImage();
    QTimer::singleShot(200, this, &RsCam::captureFrame);
    std::cout << "--------Capturing is done--------" << std::endl;
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

//------------------------------------------
void RsCam::capture_calibration_images()
{   
    Size patternsize(7,4); //interior number of corners
    vector<Point2f> corners; //this will be filled by the detected corners
    
    //lights_on();
    rs2::frameset frames;
    bool saved_ambient = true;
    bool save_calibration_image = true;
    for (int i=0; i<5; i++) 
    {
        frames = m_pipe.wait_for_frames();
        msleep(250);
    }
    

    std::map<int, std::vector<cv::Mat>> calibration_planes;
    std::map<int, std::vector<cv::Mat>> diff_planes;
    rs2::video_stream_profile rs_rgb_stream = m_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    cv::Vec3d point1(0, 0, 0);
    cv::Vec3d point2(0, 0, 1);
    cv::Vec3d normal_in_world =  point2 - point1;
    Mat normal_planes;


    cv::Mat calibration_camera_matrix;
    cv::Mat calibration_distortion_coefficients;
    cv::Mat calibration_rotation_vector;
    cv::Mat calibration_translation_vector;
    rs2::frame color_frame = frames.get_color_frame();
    auto rgb_video_frame = color_frame.as<rs2::video_frame>();
    const int rgb_width = rgb_video_frame.get_width();
    const int rgb_height = rgb_video_frame.get_height();
    rs2_intrinsics rs_rgb_instrinsics = rs_rgb_stream.get_intrinsics();

    calibration_camera_matrix = (Mat_<double>(3,3) << rs_rgb_instrinsics.fx, 0, rs_rgb_instrinsics.ppx, 0, rs_rgb_instrinsics.fy, rs_rgb_instrinsics.ppy, 0, 0, 1);
    calibration_distortion_coefficients = Mat::zeros(4, 1, DataType<double>::type);
    std::vector<Point3d> target_corner_points_world_3D;
                    for (int i = 0; i < patternsize.height; i++)
                        for (int j = 0; j < patternsize.width; j++)
                            target_corner_points_world_3D.push_back(
                                        Point3d( i * calibration_pattern_scale_, j * calibration_pattern_scale_, 0 ));
   
    while (save_calibration_image) 
    {
       
        lights_on();
        msleep(5);
        frames = m_pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat gray = Mat(color.rows, color.cols, CV_8UC1);
        cv::cvtColor(color, gray, COLOR_RGB2GRAY);

        if (saved_ambient == true)
        {
            imwrite("ambient.png", gray);
            saved_ambient = false;

        }

        lights_off();
        bool calculate_normal = true;
        for (int i = 13; i < num_lights; i += 13)
        {
            //msleep(10);
            std::cout << i <<std::endl;
            lighting(i, 255);
            
            frames = m_pipe.wait_for_frames();
            
            color_frame = frames.get_color_frame();
            Mat current_light_color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            Mat current_light_gray = Mat(color.rows, color.cols, CV_8UC1);
            cv::cvtColor(current_light_color, current_light_gray, COLOR_RGB2GRAY);
            //---------------------------------
            Mat masked_image = gray.clone();
            int all_three_planes = 0;
            while (all_three_planes<3)
            {
                msleep(10);
                bool pattern = findChessboardCorners(masked_image, patternsize, corners,
                                   CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE
                                   |CALIB_CB_FAST_CHECK);
            
                if (pattern)
                {
                    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                                TermCriteria(cv::TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));


                }else
                {
                    std::cout << "no pattern found" <<std::endl;
                    break;
                }
                   

                drawChessboardCorners(current_light_color, patternsize, Mat(corners), pattern);

                //====================== Normal Calculation
                
                // if (corners.front().y > corners.back().y)
                //     std::reverse(corners.begin(), corners.end());
            
                Point vertices[4];
                vertices[0] = corners[0];
                vertices[1] = corners[(patternsize.height-1)*patternsize.width];
                vertices[2] = corners[(patternsize.height-1)*patternsize.width+(patternsize.width-1)];
                vertices[3] = corners[patternsize.width-1];
                cv::fillConvexPoly(masked_image, vertices, 4, Scalar(0,255,255));
                cv::imshow("test" + std::to_string(all_three_planes), masked_image);
                Mat currentplane;
                Mat diff;
                Mat mask(gray.size(), CV_8U, Scalar(0));
                cv::fillConvexPoly(mask, vertices, 4, Scalar(255,255,255));
                cv::bitwise_and(current_light_gray, mask, currentplane);
                cv::bitwise_and(current_light_gray - gray, mask, diff);
                //cv::imshow("curret masked" + std::to_string(all_three_planes), currentplane);
                calibration_planes[all_three_planes].push_back(currentplane);
                diff_planes[all_three_planes].push_back(diff);
                all_three_planes = all_three_planes + 1;

                if (calculate_normal)
                {
            

                    bool found_pose = solvePnP(target_corner_points_world_3D, corners, calibration_camera_matrix, calibration_distortion_coefficients, calibration_rotation_vector, calibration_translation_vector);
                    if (found_pose) 
                    {
                        Mat R;
                        Rodrigues(calibration_rotation_vector, R); // Convert "rvec arg" m_calibration_rotation_vector to 3x3 rotation matrix R

                        // BUILD WORLD TO CAMERA TRANSFORMATION MATRIX
                        Mat w_M_c_ = Mat::eye(4, 4, R.type()); // M is 4x4
                        w_M_c_( Range(0,3), Range(0,3) ) = R * 1; // copies R into M
                        w_M_c_( Range(0,3), Range(3,4) ) = calibration_translation_vector * 1;
                        
                        cv::Mat normal_currentplane = R * Mat(normal_in_world);
                        normal_planes.push_back(normal_currentplane);
                    }

                //======================
                }
            
            }
            std::cout << "masked are saved"<< std::endl;
            calculate_normal = false;
            //---------------------------------
            // light_map[i] = current_light_color;
            // std::string filename = "cap_test_light"+ std::to_string(i) + ".png";
            // Mat diff = current_light_gray - gray;
            // std::string filename1 = "cap_test_light_diff"+ std::to_string(i) + ".png";
            // if (save_calibration_image)
            // {
            //     std::cout << "saving:-->>" <<std::endl;
            //     imwrite(filename, current_light_gray);
            //     imwrite(filename1, diff);
            // }
           
            lights_off();
            emit newCamFrame(current_light_color.clone());
        }
    
        save_calibration_image = false;

    }
    std::cout << "Calibration Images Saved" << std::endl;
    std::cout << calibration_planes[2].size() << std::endl;
    for (int i = 0; i < calibration_planes[2].size(); i++)
    {
        cv::imshow("plane" , calibration_planes[2][i]);
        cv::waitKey(200);

    }

    for (int i = 0; i < calibration_planes[2].size(); i++)
    {
        cv::imshow("diff plane", diff_planes[2][i]);
        cv::waitKey(200);
    }
    Calibration::withThreePlane(normal_planes, calibration_planes);
}

//------------------------------------------
void RsCam::captureAmbientImage() {

    rs2::frameset frames ;

    msleep(250);
    lights_off();
    for (int i=0; i<5; i++) {
        frames = m_pipe.wait_for_frames();
        msleep(250);
    }

    vector<Vec3f> circles;
    double c_min_d = MAXFLOAT;
    int c_min_i = -1;
    double accuracy = 1.0;

    cv::Vec3d point1(0, 0, 0);
    cv::Vec3d point2(0, 0, 1);
    cv::Vec3d normal_in_world =  point2 - point1;
    std::cout << "normal in world" <<normal_in_world<<std::endl;
    std::vector<Mat> masked_images (0);
    Mat normal_planes;
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
    
        while (true) {
            msleep(10);
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
           
            if (masked_images.size() < 3)
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
                                    Point3d( i * calibration_pattern_scale_, j * calibration_pattern_scale_, 0 ));

                

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
                    
                    cv::Mat normal_currentplane = R * Mat(normal_in_world);
                    // --------------------
                    Vec4d point2_in_world_hom(0, 0, 1.0, 1.0);
                    Vec4d point1_in_world_hom(0, 0, 0, 1.0);
                    Mat point2_in_camera_hom = w_M_c_ * point2_in_world_hom;
                    Mat point1_in_camera_hom = w_M_c_ * point1_in_world_hom;
                    //std::cout << point2_in_camera_hom<<std::endl;
                    // Point3d world_point_3d(point2_in_camera_hom.at<double>(0), 
                    //                     point2_in_camera_hom.at<double>(1), point2_in_camera_hom.at<double>(2));
                    cv::Mat point2_on_camera = (cv::Mat_<double>(1, 3) << point2_in_camera_hom.at<double>(0), 
                    point2_in_camera_hom.at<double>(1), point2_in_camera_hom.at<double>(2));

                    cv::Mat point1_on_camera = (cv::Mat_<double>(1, 3) << point1_in_camera_hom.at<double>(0), 
                    point1_in_camera_hom.at<double>(1), point1_in_camera_hom.at<double>(2));


                    cv::Mat normal_in_camera = point2_on_camera - point1_on_camera;
                    
                    //cv::normalize(normal_in_camera, normal_in_camera, 1,cv::NORM_L2);
                    cout << "normals by applying the whole transformation" << endl << " "  <<normal_in_camera << endl << endl;
                    cout << "normals by applying rotation" << endl << " "  << normal_currentplane.t() << endl << endl;
                    
                    //---------------------
                    
                    normal_planes.push_back(normal_in_camera);


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

            if (masked_images.size()==3 && get_plane_normal)
            {
               
                Calibration::get_plane_normals(normal_planes);
                get_plane_normal = false;
            }


            if (masked_images.size()==3 && save_masked)
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
    imwrite("ambient_hand.png", ambientImage);
    //cv::imshow("test", ambientImage);

}
void RsCam::captureAmbientImage_new()
{
    lights_off();
    msleep(250);
    rs2::frameset frames ;
    for (int i=0; i<5; i++) {
        frames = m_pipe.wait_for_frames();
        msleep(250);
    }

    frames = m_pipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();
    Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    cvtColor(color, ambientImage, COLOR_RGB2GRAY);
    imwrite("ambient_hand.png", ambientImage);
    avgImgIntensity = cv::mean(ambientImage)[0];

}

void RsCam::calibrate() {
    std::cout << "calibrate hit"<<std::endl;
    isCalibrating = true;
    capture_calibration_images();
    isCalibrating = false;
    //QTimer::singleShot(20, this, &RsCam::captureFrame);
}


// void RsCam::captureFrame() {
    
//     std::cout<< "capture frame" <<std::endl;
//     if (isCalibrating)
//         return;

//     currentLight++;
//     if (currentLight==num_lights)
//         currentLight = 0;

//     if (!has_ambient || currentLight==num_lights) {
//         std::cout<< "taking ambient" <<std::endl;
//         captureAmbientImage_new();
//         has_ambient = true;
//         currentLight = 0;
//     }

//     Mat camFrame(m_rgb_height, m_rgb_width, CV_8UC1);
//     imgIdx = (imgIdx+1) % 8;

//     if (testMode) {
//         camFrame = testImages[imgIdx].clone();
//         /* faking camera image acquisition time */
//         eventLoopTimer->setInterval(1000/FRAME_RATE);
//     } else {
//         double total_lights = 104;
//         double mid_strip = 52;
//         int light_id = int( (total_lights-mid_strip)/2 + currentLight*(mid_strip/8));
//         std::cout<< "light id" <<light_id<<std::endl;
//         lighting(light_id, 255);
//         msleep(20);
//         rs2::frameset frames = m_pipe.wait_for_frames();
//         lighting(light_id, 0);
//         rs2::frame color_frame = frames.get_color_frame();
//         Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
//         cvtColor(color, camFrame, COLOR_RGB2GRAY);

//         camFrame -= ambientImage;
//         double minVal;
//         double maxVal;
//         Point minLoc;
//         Point maxLoc;
//         Point center = Point(calibrationTarget[0], calibrationTarget[1]);
//         Mat mask = Mat::zeros(color.rows, color.cols, CV_8UC1);
//         int radius = calibrationTarget[2];
//         int target_radius = int(radius*0.65);
//         Rect target_roi(center.x-target_radius,center.y-target_radius,target_radius*2,target_radius*2);
//         rectangle(color, target_roi, Scalar(255,0,0));
//         mask(target_roi) = 255;
//         minMaxLoc( camFrame, &minVal, &maxVal, &minLoc, &maxLoc, mask );
//         circle(color, maxLoc, 2, Scalar(255,0,0), 1);

//         emit newCamFrame(color.clone());

//         calibrationImages[currentLight] = camFrame;

//     }

void RsCam::captureFrame() 
{
   
    std::cout<< "capture frame" <<std::endl;
    if (isCalibrating)
        return;

    if (!has_ambient || currentLight==num_lights) {
        std::cout<< "taking ambient" <<std::endl;
        captureAmbientImage_new();
        has_ambient = true;
        currentLight = 0;
    }

    // if (cropped_need)
    // {
    //     set_width_height();
    //     cropped_need = false;
    //     // rs2::frameset frames;
    //     // for (int i=0; i<5; i++) 
    //     // {
    //     //     frames = m_pipe.wait_for_frames();
    //     //     msleep(250);
    //     // }
    //     // rs2::frame color_frame = frames.get_color_frame();
    //     // Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    //     // Mat grey;
    //     // cvtColor(color, grey, COLOR_RGB2GRAY);
    //     // namedWindow(WINDOW_NAME);
	//     // Mat tempImage;
    //     // setMouseCallback(WINDOW_NAME, on_MouseHandle, (void*) &grey);
        
	//     // while (1) {
    //     //     color.copyTo(tempImage);
            
    //     //     if (g_bDrawingBox)
    //     //         DrawRectangle(tempImage, g_rectangle);
    //     //     imshow(WINDOW_NAME, tempImage);
    //     //     //char c = getchar();

    //     //     if (complete_rect==false)  // stop drawing rectanglge if the key is 'ESC'
    //     //         break;

	//     // }
    //     // cropped_rect = g_rectangle;
    //     // if (!complete_rect)
    //     // {
    //     //     cv::Mat croppedImage = color(cropped_rect);
    //     //     cv::imshow("cropped", croppedImage);
    //     //     cropped_need = false;
    //     //     //cv::destroyAllWindows();
    //     // }
        
        
    // }
    Mat camFrame(m_rgb_height, m_rgb_width, CV_8UC1);


    if (testMode) {
        camFrame = testImages[imgIdx].clone();
        /* faking camera image acquisition time */
        eventLoopTimer->setInterval(1000/FRAME_RATE);
    } else 
    {

        if (id < 8)
        {
            light_id = light_idx[id];
            std::cout << light_id << std::endl;
            lighting(light_id, 255);
            
            msleep(20);
            rs2::frameset frames = m_pipe.wait_for_frames();
            rs2::frame color_frame = frames.get_color_frame();
            Mat color(Size(m_rgb_width, m_rgb_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            cvtColor(color, camFrame, COLOR_RGB2GRAY);

        
            emit newCamFrame(color.clone());

            captured_images[light_id] = camFrame;
            //captured_images[light_id]= camFrame(cropped_rect);
            camFrame -= ambientImage;
            captured_diff[light_id] = camFrame;
            //captured_diff[light_id] = camFrame(cropped_rect);
            if (save_screenshot)
            {
                imwrite("capture" + std::to_string(light_id) + ".png", camFrame);
                imwrite("capture_diff" + std::to_string(light_id) + ".png", camFrame);
                
            }
            
            //light_id = light_id + 13;
            QTimer::singleShot(20, this, &RsCam::captureFrame);
            lights_off();
            id = id + 1;
            //
        }
        else
        {
            std::cout << "capturing is done" <<std::endl;
            save_screenshot = false;
            // ps = new PhotometricStereo(m_rgb_width, m_rgb_height, avgImageIntensity());
            // ps->execute_new(captured_diff, ambientImage);
            emit newFrames(captured_diff);
            //light_id = 13;
            id = 0;

            QTimer::singleShot(20, this, &RsCam::captureFrame);
        }
    }   
       
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
void RsCam::test_get_normal()
{
    get_plane_normal = true;   
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
void RsCam::on_MouseHandle(int event, int x, int y, int flags, void* param) {
	Mat& image = *(cv::Mat*) param;
	switch (event) {
	case EVENT_MOUSEMOVE: {    // When mouse moves, get the current rectangle's width and height
		if (g_bDrawingBox) {
			g_rectangle.width = x - g_rectangle.x;
			g_rectangle.height = y - g_rectangle.y;
            std::cout << "Rectangle continues: " << g_rectangle << std::endl;
			DrawRectangle(image, g_rectangle);
            
		}
	}
		break;
	case EVENT_LBUTTONDOWN: {  // when the left mouse button is pressed down,
		                       //get the starting corner's coordinates of the rectangle
		g_bDrawingBox = true;
		g_rectangle = Rect(x, y, 0, 0);
        std::cout << "Rectangle start: " << g_rectangle << std::endl;
	}
		break;
	case EVENT_LBUTTONUP: {   //when the left mouse button is released,
		                      //draw the rectangle
		g_bDrawingBox = false;
		if (g_rectangle.width < 0) {
			g_rectangle.x += g_rectangle.width;
			g_rectangle.width *= -1;
		}

		if (g_rectangle.height < 0) {
			g_rectangle.y += g_rectangle.height;
			g_rectangle.height *= -1;
		}
        std::cout << "Rectangle ended: " << g_rectangle << std::endl;
        complete_rect = false;
        
		DrawRectangle(image, g_rectangle);
        break;
	}
		break;
	}
}
void RsCam::DrawRectangle(Mat& img, Rect box)
{
     RNG g_rng(0); 
	//Draw a rectangle with random color
	cv::rectangle(img, box.tl(), box.br(), Scalar(g_rng.uniform(0, 255),
					g_rng.uniform(0,255),g_rng.uniform(0,255)));

    
}