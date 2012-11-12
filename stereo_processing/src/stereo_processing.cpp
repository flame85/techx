#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Float64MultiArray.h"
#include <string>
using namespace std;
using namespace cv;
/*void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  try
  {
    cvShowImage("view", bridge.imgMsgToCv(msg, "bgr8"));
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}*/
#define DEBUG 0
#define USE_SGBM 1
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
cv_bridge::CvImagePtr cv_ptr_left;
cv_bridge::CvImagePtr cv_ptr_right;
cv::Mat left_img;
cv::Mat right_img;
Mat Q, disp, dispReal, xyz, disp8;
Size img_size;
StereoSGBM sgbm;
StereoBM bm;

#if(DEBUG)
//float dispValue, p3dZ, p3dX, p3dY;
Point pt2dl, pt2dr;
float dispshift;
#endif
cv_bridge::CvImage m_cvi;
image_transport::Publisher m_image_right_pub;
image_transport::Publisher m_image_left_pub;
image_transport::Publisher m_disparity_pub;
sensor_msgs::Image m_left_msg;
sensor_msgs::Image m_right_msg;
sensor_msgs::Image m_disparity_msg;
double start=0.; 
int frame_counter = 0;

int SADWindowSize = 0, numberOfDisparities = 0;
inline double getSecOfNow()
{
    timeval tmpCurTime;
    gettimeofday(&tmpCurTime, NULL);
    return ((double)tmpCurTime.tv_usec / 1000000.) + ((double)tmpCurTime.tv_sec);
}
#if(DEBUG)
void my_mouse_callback( int event, int x, int y, int flags, void* param ){
  
  switch( event ){
    case CV_EVENT_MOUSEMOVE: 
      
      break;
      
    case CV_EVENT_LBUTTONDOWN:
      pt2dl.x = x;
      pt2dl.y = y;
      printf("2d cord (%d %d), 3d pose (%f %f %f)\n", x, y, 
                                                      xyz.at<Vec3f>(y,x)[0], 
                                                      xyz.at<Vec3f>(y,x)[1], 
                                                      xyz.at<Vec3f>(y,x)[2]);
#if(USE_SGBM)
      dispshift = dispReal.at<float>(y,x);
#else
      dispshift = disp.at<float>(y,x);
#endif
      if(dispshift < 0)
      {
        printf("no disparity!\n");
        return;
      }
      pt2dr.x = pt2dl.x - (int)(dispshift+0.5);  
      pt2dr.y = pt2dl.y;  
      printf("dispshift %f lx %d, rx %d\n", dispshift, pt2dl.x, pt2dr.x);
      /*dispValue = dispshift;
      p3dZ = 527.37693*-0.11991441/dispValue; 
      p3dX = ((float)x - 318.02784) * p3dZ / 527.37693;
      p3dY = ((float)y - 240.42265) * p3dZ / 527.37693;
      printf("dispValue is %d, disp8 is %d 3d coord is (%f %f %f)\n",(disp.at<unsigned short>(y,x)), disp8.at<unsigned char>(y,x), p3dX, p3dY, p3dZ);
      */
      break;
      
    case CV_EVENT_LBUTTONUP:
      break;
  }
}
#endif

void qMatCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(!Q.empty())
    return;
  Q = (Mat_<double>(4,4) << msg->data[0], msg->data[1], msg->data[2], msg->data[3],
      msg->data[4], msg->data[5], msg->data[6], msg->data[7],
      msg->data[8], msg->data[9], msg->data[10], msg->data[11],
      msg->data[12], msg->data[13], msg->data[14], msg->data[15]);
  std::cout << Q << std::endl;
}

void imageCallback (const sensor_msgs::ImageConstPtr& left_img_msg, 
                    const sensor_msgs::ImageConstPtr& right_img_msg)
{
  //printf("receive images\n");

  try
  {
    cv_ptr_left = cv_bridge::toCvCopy(left_img_msg, "bgr8");
    cv_ptr_right = cv_bridge::toCvCopy(right_img_msg, "bgr8");
    left_img = cv_ptr_left->image;
    right_img = cv_ptr_right->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if(frame_counter == 0)
  {
    start = getSecOfNow();
  }

#if(USE_SGBM)
    sgbm(left_img, right_img, disp);
    disp.convertTo(dispReal, CV_32F, 1.0/16.0);
    reprojectImageTo3D(dispReal, xyz, Q, true);
#else
    cvtColor(left_img, left_img, CV_RGB2GRAY);
    cvtColor(right_img, right_img, CV_RGB2GRAY);
    bm(left_img, right_img, disp, CV_32F);
    //disp.convertTo(dispReal, CV_32F);
    reprojectImageTo3D(disp, xyz, Q, true);
#endif
#if(DEBUG)
    disp.convertTo(disp8, CV_8U);
    cv::circle(left_img, pt2dl, 3, CV_RGB(255,0,0), -1);
    cv::circle(right_img, pt2dr, 3, CV_RGB(255,0,0), -1);
    cv::circle(disp8, pt2dl, 3, CV_RGB(255,0,0), -1);
    /*cv::circle(imglr, pt2dl, 3, CV_RGB(255,0,0), -1);
    cv::circle(imgrr, pt2dr, 3, CV_RGB(255,0,0), -1);
    cv::circle(disp8, pt2dl, 3, CV_RGB(255,0,0), -1);*/
    //printf("image callback use time %lf sec\n", getSecOfNow() - start);
    imshow("disparity", disp8);
    imshow("left", left_img);
    imshow("right", right_img);
    cv::waitKey(10);
#endif
    //publish disparity image
    ros::Time time_now = ros::Time::now();
    m_cvi.header.stamp = time_now;
    m_cvi.header.frame_id = "/camera";
    m_cvi.encoding = "";
#if(USE_SGBM)
    m_cvi.image = dispReal;
#else
    m_cvi.image = disp;
    //cout << disp << endl;
#endif
    m_cvi.toImageMsg(m_disparity_msg);
    m_disparity_pub.publish(m_disparity_msg);
    frame_counter++;
    if(frame_counter == 20)
    {
      double time_elapsed = getSecOfNow() - start;
      printf("publisheing msgs in %lf hz\n", (double)frame_counter / time_elapsed);
      frame_counter = 0;
    }
}

int main(int argc, char **argv)
{
  img_size.width = 640;
  img_size.height = 480;
  numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
#if(USE_SGBM)
  int cn = left_img.channels();
  sgbm.preFilterCap = 63;
  sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
  sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.minDisparity = 0;
  sgbm.numberOfDisparities = numberOfDisparities;
  sgbm.uniquenessRatio = 10;
  sgbm.speckleWindowSize =100;
  sgbm.speckleRange = 32;
  sgbm.disp12MaxDiff = 1;
  sgbm.fullDP = false;  
#else
  //bm.state->roi1 = roi1;
  //bm.state->roi2 = roi2;
  bm.state->preFilterCap = 31;
  bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
  bm.state->minDisparity = 0;
  bm.state->numberOfDisparities = numberOfDisparities;
  bm.state->textureThreshold = 10;
  bm.state->uniquenessRatio = 15;
  bm.state->speckleWindowSize = 100;
  bm.state->speckleRange = 32;
  bm.state->disp12MaxDiff = 1;
#endif
  //ros init
#if(DEBUG)
  namedWindow( "disparity" );
  namedWindow( "left" );
  setMouseCallback( "disparity", my_mouse_callback);
  setMouseCallback( "left", my_mouse_callback);
#endif
  ros::init(argc, argv, "image_listener");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/q_matrix", 1, qMatCallback);
  image_transport::ImageTransport m_it(nh);
  m_disparity_pub = m_it.advertise("/camera/disparity", 1);
  //cvNamedWindow("view");
  //cvStartWindowThread();
  //image_transport::ImageTransport it(nh);
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, "/left_rectified/rgb_rectified", 1);
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub(nh, "/right_rectified/rgb_rectified", 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), left_image_sub, right_image_sub);
  sync.registerCallback(boost::bind(imageCallback,  _1, _2));
  //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
}
