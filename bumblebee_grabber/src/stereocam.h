#ifndef STEREOCAM_H
#define STEREOCAM_H
#include <opencv2/opencv.hpp>
#include "pgr_stereocam.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
using namespace std;
using namespace cv;
class stereoCam {
    public:
        stereoCam(char* intrinsic_filename, char* extrinsic_filename);
        ~stereoCam();
        void grabRGBs();
        void publishImagePairs();
        Mat m_rightRGB, m_leftRGB;
        Mat m_smlRightRGB, m_smlLeftRGB;
    private:
        PGRStereoCamera_t stereoCamera;
        dc1394camera_t* camera;
        dc1394error_t err;
        dc1394_t* d;
        dc1394camera_list_t* list;
        unsigned int nThisCam;
        // allocate a buffer to hold the de-interleaved images
        unsigned char* pucDeInterlacedBuffer;
        unsigned char* pucRGBBuffer;
        unsigned char* pucGreenBuffer; // allocate a buffer to hold the de-interleaved images
        // unsigned char*
        unsigned char* pucRightRGB;
        unsigned char* pucLeftRGB;
        unsigned char* pucCenterRGB;
        ros::NodeHandle m_nh;  // private node handle
        image_transport::ImageTransport *m_it;
        cv_bridge::CvImage m_cvi;
        image_transport::CameraPublisher m_rgb_right_pub;
        image_transport::CameraPublisher m_rgb_left_pub;
        image_transport::CameraPublisher m_rgb_right_rec_pub;
        image_transport::CameraPublisher m_rgb_left_rec_pub;
        image_transport::CameraPublisher m_rgb_big_right_rec_pub;
        image_transport::CameraPublisher m_rgb_big_left_rec_pub;
        ros::Publisher m_Q_pub;
        std_msgs::Float64MultiArray m_Q_msg;
        sensor_msgs::Image m_left_msg;
        sensor_msgs::Image m_right_msg;
        sensor_msgs::Image m_left_rec_msg;
        sensor_msgs::Image m_right_rec_msg;
        sensor_msgs::Image m_big_left_rec_msg;
        sensor_msgs::Image m_big_right_rec_msg;
        sensor_msgs::CameraInfo m_left_cam_info;
        sensor_msgs::CameraInfo m_right_cam_info;
        sensor_msgs::CameraInfo m_left_rec_cam_info;
        sensor_msgs::CameraInfo m_right_rec_cam_info;
        sensor_msgs::CameraInfo m_big_left_rec_cam_info;
        sensor_msgs::CameraInfo m_big_right_rec_cam_info;
        string m_frame_id;
        string m_camera_name;


        Mat M1, D1, M2, D2, R, T, R1, P1, R2, P2, Q, m_leftRec, m_rightRec, m_bigLeftRec, m_bigRightRec;
        Mat BM1, BD1, BM2, BD2, BR, BT, BR1, BP1, BR2, BP2, BQ;
        Mat map11, map12, map21, map22;
        Mat bmap11, bmap12, bmap21, bmap22;
        Rect roi1, roi2, broi1, broi2;
        Size img_size, bimg_size;
        Mat imglr, imgrr; // rectified images

        //private func
        void getInitParams();
        void cleanup_and_exit( dc1394camera_t* camera );
        void initCameraInfo();
};
#endif
