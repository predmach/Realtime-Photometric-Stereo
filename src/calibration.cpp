#include "calibration.h"

void Calibration::withFourPlanes() {
    
    std::cout << "Calibration:" << std::endl;
    
    /* reading 8 x 4 plane images */
    std::vector<cv::Mat> dhImages, lhImages, rhImages, uhImages;
    std::cout << "..reading images" << std::endl;
	for (int i = 0; i < 8; i++) {
        std::stringstream s1, s2, s3, s4;
        s1 << PATH_ASSETS << "inclinedPlane/down-high/image" << i << ".png";
        s2 << PATH_ASSETS << "inclinedPlane/left-high/image" << i << ".png";
        s3 << PATH_ASSETS << "inclinedPlane/right-high/image" << i << ".png";
        s4 << PATH_ASSETS << "inclinedPlane/up-high/image" << i << ".png";
        cv::Mat img1 = cv::imread(s1.str(), cv::ImreadModes::IMREAD_GRAYSCALE);
        cv::Mat img2 = cv::imread(s2.str(), cv::ImreadModes::IMREAD_GRAYSCALE);
        cv::Mat img3 = cv::imread(s3.str(), cv::ImreadModes::IMREAD_GRAYSCALE);
        cv::Mat img4 = cv::imread(s4.str(), cv::ImreadModes::IMREAD_GRAYSCALE);
        
        /* using cropped images as provided by camera class */
        assert(img1.rows == IMG_HEIGHT && img1.cols == IMG_HEIGHT);
        assert(img2.rows == IMG_HEIGHT && img2.cols == IMG_HEIGHT);
        assert(img3.rows == IMG_HEIGHT && img3.cols == IMG_HEIGHT);
        assert(img4.rows == IMG_HEIGHT && img4.cols == IMG_HEIGHT);
        dhImages.push_back(img1);
        lhImages.push_back(img2);
        rhImages.push_back(img3);
        uhImages.push_back(img4);
	}
    
    float a = 1.7922437549939767;
    float c = 25.97710530447917;
    int channels = 24;
    int numImgs = dhImages.size();
    int imgHeight = dhImages[0].rows;
    int imgWidth = dhImages[0].cols;
    
    cv::Mat N = (cv::Mat_<float>(4,3) <<    -a, 0, c,
                                             0,-a, c,
                                             a, 0, c,
                                             0, a, c);
    cv::normalize(N, N, 2, cv::NORM_L2);
        
    cv::Mat Ninv;
    cv::invert(N, Ninv, cv::DECOMP_SVD);
    cv::Mat S(imgHeight, imgWidth, CV_32FC(channels));
    
    std::cout << "..calculating light direction matrix" << std::endl;
    for (int y=0; y<imgHeight; y++) {
        for (int x=0; x<imgWidth; x++) {
            /* local light direction matrix */
            cv::Mat s(8, 3, CV_32F, cv::Scalar::all(0));
            /* filling up matrix */
            for (int i=0; i<numImgs; i++) {
                cv::Vec4f I(rhImages[i].at<uchar>(y,x),
                            dhImages[i].at<uchar>(y,x),
                            lhImages[i].at<uchar>(y,x),
                            uhImages[i].at<uchar>(y,x));
                cv::Mat svec = Ninv * cv::Mat(I);
                //cv::normalize(svec, svec);
                /* offset: (row * numCols * numChannels) + (col * numChannels) + (channel) */
                ((float*)s.data)[(i*3*1)+(0*1)+(0)] = ((float*)svec.data)[(0*1*1)+(0*1)+(0)];
                ((float*)s.data)[(i*3*1)+(1*1)+(0)] = ((float*)svec.data)[(1*1*1)+(0*1)+(0)];
                ((float*)s.data)[(i*3*1)+(2*1)+(0)] = ((float*)svec.data)[(2*1*1)+(0*1)+(0)];
            }
            /* pseudo-inverse of local light matrix */
            cv::invert(s, s, cv::DECOMP_SVD);
            for (int i=0; i<numImgs; i++) {
                ((float*)S.data)[(y*imgWidth*channels)+(x*channels)+(i*3+0)] = ((float*)s.data)[(0*8*1)+(i*1)+(0)];
                ((float*)S.data)[(y*imgWidth*channels)+(x*channels)+(i*3+1)] = ((float*)s.data)[(1*8*1)+(i*1)+(0)];
                ((float*)S.data)[(y*imgWidth*channels)+(x*channels)+(i*3+2)] = ((float*)s.data)[(2*8*1)+(i*1)+(0)];
            }
            
        }
    }
    
    /* saving light matrix */
    FILE *pFile;
    std::stringstream s;
    s << PATH_ASSETS << "lightMat.kaw";
    std::cout << "..writing to file: " << s.str() << std::endl;
    pFile = fopen(s.str().c_str(), "wb");
    fwrite(S.data, 1, sizeof(float)*S.rows*S.cols*S.channels(), pFile);
    fclose(pFile);
}
void Calibration::withThreePlane(cv::Mat normals, std::map<int, std::vector<cv::Mat>> calibration_planes) 

{
    
 
    std::cout << "Calibration With Three Plane:" << std::endl;
    
    /* reading 8 x 3 plane images */
    std::vector<cv::Mat> plane1, plane2, plane3;
    std::cout << "..reading images" << std::endl;
	for (int i = 0; i < 8; i++) {
        std::stringstream s1, s2, s3;
        // s1 << PATH_ASSETS << "build/plane1_capture_" << i << ".png";
        // s2 << PATH_ASSETS << "build/plane2_capture_" << i << ".png";
        // s3 << PATH_ASSETS << "build/plane3_capture_" << i << ".png";
        
        // cv::Mat img1 = cv::imread(s1.str(), cv::ImreadModes::IMREAD_GRAYSCALE);
        // cv::Mat img2 = cv::imread(s2.str(), cv::ImreadModes::IMREAD_GRAYSCALE);
        // cv::Mat img3 = cv::imread(s3.str(), cv::ImreadModes::IMREAD_GRAYSCALE);

        cv::Mat img1 = calibration_planes[0][i];
        cv::Mat img2 = calibration_planes[1][i];
        cv::Mat img3 = calibration_planes[2][i];
        
        /* using cropped images as provided by camera class */
        // assert(img1.rows == IMG_HEIGHT && img1.cols == IMG_HEIGHT);
        // assert(img2.rows == IMG_HEIGHT && img2.cols == IMG_HEIGHT);
        // assert(img3.rows == IMG_HEIGHT && img3.cols == IMG_HEIGHT);
        plane1.push_back(img1);
        plane2.push_back(img2);
        plane3.push_back(img3);
    }
    
    std::cout << "images successfully loaded" <<std::endl;
    // float a = 1.7922437549939767; // The whole materix N is not clear for me. 
    // float c = 25.97710530447917;
    int channels = 24;
    int numImgs = plane1.size();
    int imgHeight = plane1[0].rows;
    int imgWidth = plane1[0].cols;
    
    // cv::Mat N = (cv::Mat_<float>(4,3) <<    -a, 0, c,
    //                                          0,-a, c,
    //                                          a, 0, c,
    //                                          0, a, c);
    cv::Mat N = (cv::Mat_<double>(3,3) <<    normals.at<double>(0,0), normals.at<double>(0,1), normals.at<double>(0,2),
                                             normals.at<double>(0,3),normals.at<double>(0,4), normals.at<double>(0,5),
                                             normals.at<double>(0,6), normals.at<double>(0,7), normals.at<double>(0,8));


    std::cout << "normals N: " << N << std::endl;
    cv::normalize(N, N, 2, cv::NORM_L2);
        
    cv::Mat Ninv;
    cv::invert(N, Ninv, cv::DECOMP_SVD);
    cv::Mat S(imgHeight, imgWidth, CV_32FC(channels));
    
    std::cout << "..calculating light direction matrix" << std::endl;
    for (int y=0; y<imgHeight; y++) {
        for (int x=0; x<imgWidth; x++) {
            /* local light direction matrix */
            cv::Mat s(8, 3, CV_32F, cv::Scalar::all(0));
            /* filling up matrix */
            for (int i=0; i<numImgs; i++) {
                cv::Vec3f I(plane1[i].at<uchar>(y,x),
                            plane2[i].at<uchar>(y,x),
                            plane3[i].at<uchar>(y,x));
                std::cout << I << std::endl;
                std::cout << Ninv << std::endl;
                cv::Mat svec = Ninv * cv::Mat(I);
                //cv::normalize(svec, svec);
                /* offset: (row * numCols * numChannels) + (col * numChannels) + (channel) */
                ((float*)s.data)[(i*3*1)+(0*1)+(0)] = ((float*)svec.data)[(0*1*1)+(0*1)+(0)];
                ((float*)s.data)[(i*3*1)+(1*1)+(0)] = ((float*)svec.data)[(1*1*1)+(0*1)+(0)];
                ((float*)s.data)[(i*3*1)+(2*1)+(0)] = ((float*)svec.data)[(2*1*1)+(0*1)+(0)];
            }
            /* pseudo-inverse of local light matrix */
            cv::invert(s, s, cv::DECOMP_SVD);
            for (int i=0; i<numImgs; i++) {
                ((float*)S.data)[(y*imgWidth*channels)+(x*channels)+(i*3+0)] = ((float*)s.data)[(0*8*1)+(i*1)+(0)];
                ((float*)S.data)[(y*imgWidth*channels)+(x*channels)+(i*3+1)] = ((float*)s.data)[(1*8*1)+(i*1)+(0)];
                ((float*)S.data)[(y*imgWidth*channels)+(x*channels)+(i*3+2)] = ((float*)s.data)[(2*8*1)+(i*1)+(0)];
            }
            
        }
    }
    
    // /* saving light matrix */
    // FILE *pFile;
    // std::stringstream s;
    // s << PATH_ASSETS << "lightMat.kaw";
    // std::cout << "..writing to file: " << s.str() << std::endl;
    // pFile = fopen(s.str().c_str(), "wb");
    // fwrite(S.data, 1, sizeof(float)*S.rows*S.cols*S.channels(), pFile);
    // fclose(pFile);
}
void Calibration::get_plane_normals(cv::Mat normals)
{
    cv::Mat N = (cv::Mat_<double>(3,3) <<    normals.at<double>(0,0), normals.at<double>(0,1), normals.at<double>(0,2),
                                             normals.at<double>(0,3),normals.at<double>(0,4), normals.at<double>(0,5),
                                             normals.at<double>(0,6), normals.at<double>(0,7), normals.at<double>(0,8));


    std::cout << "normals N = " << std::endl << " "  << N << std::endl << std::endl;
    std::cout << "normals N * N_transpose = " << std::endl << " "  << N * N.t() << std::endl << std::endl;
    
    

}
bool Calibration::saveMatBinary(const std::string& filename, const cv::Mat& mat){
    std::ofstream ofs(filename, std::ios::binary);
    return writeMatBinary(ofs, mat);
}
bool Calibration::writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat)
{
    if(!ofs.is_open()){
        return false;
    }
    if(out_mat.empty()){
        int s = 0;
        ofs.write((const char*)(&s), sizeof(int));
        return true;
    }
    int type = out_mat.type();
    ofs.write((const char*)(&out_mat.rows), sizeof(int));
    ofs.write((const char*)(&out_mat.cols), sizeof(int));
    ofs.write((const char*)(&type), sizeof(int));
    ofs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

    return true;
}
