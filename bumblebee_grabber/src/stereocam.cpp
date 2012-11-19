#include "stereocam.h"
#include <tf/transform_listener.h>
stereoCam::stereoCam(char* intrinsic_filename, char* extrinsic_filename) 
{
    d = dc1394_new ();
    err = dc1394_camera_enumerate (d, &list);
    if (list->num == 0)
    {
        printf("No cameras found!\n");
        exit(1);
    }
    for ( nThisCam = 0; nThisCam < list->num; nThisCam++ )
    {
        camera = dc1394_camera_new(d, list->ids[nThisCam].guid);
        if(!camera)
            printf("Failed to initialize camera with guid %llux", list->ids[nThisCam].guid);
        printf( "Camera %d model = '%s'\n", nThisCam, camera->model );
        if ( isStereoCamera(camera)) {
            printf( "Using this camera\n" );
            break;
        }
        dc1394_camera_free(camera);
    }
    dc1394_camera_free_list (list);

    err = queryStereoCamera( camera, &stereoCamera );

    //dc1394_v
    if ( stereoCamera.nBytesPerPixel != 2 ){
        // can't handle XB3 3 bytes per pixel
        fprintf( stderr, "Example has not been updated to work with XB3 in 3 camera mode yet!\n" );
        cleanup_and_exit( stereoCamera.camera );
    }

    // set the capture mode
    printf( "Setting stereo video capture mode\n" );
    err = setStereoVideoCapture( &stereoCamera );
    if ( err != DC1394_SUCCESS ){
        fprintf( stderr, "Could not set up video capture mode\n" );
        cleanup_and_exit( stereoCamera.camera );
    }

    // have the camera start sending us data
    printf( "Start transmission\n" );
    err = startTransmission( &stereoCamera );
    if ( err != DC1394_SUCCESS){
        fprintf( stderr, "Unable to start camera iso transmission\n" );
        cleanup_and_exit( stereoCamera.camera );
    }

    unsigned int nBufferSize = stereoCamera.nRows*stereoCamera.nCols*stereoCamera.nBytesPerPixel;
    //// allocate a buffer to hold the de-interleaved images
    pucDeInterlacedBuffer = new unsigned char[ nBufferSize ];
    pucRGBBuffer 	= new unsigned char[ 3 * nBufferSize ];
    pucGreenBuffer 	= new unsigned char[ nBufferSize ];

    pucRightRGB = new unsigned char[3*nBufferSize];
    pucLeftRGB = new unsigned char[3*nBufferSize];
    pucCenterRGB = new unsigned char[3*nBufferSize];

    m_rightRGB.create(stereoCamera.nRows, stereoCamera.nCols, CV_8UC3);
    m_leftRGB.create(stereoCamera.nRows, stereoCamera.nCols, CV_8UC3);
    //m_nh = ros::NodeHandle("~");
    m_it = new image_transport::ImageTransport(m_nh);
    m_rgb_left_pub = m_it->advertiseCamera("/left_raw/rgb_raw", 1);
    m_rgb_right_pub = m_it->advertiseCamera("/right_raw/rgb_raw", 1);
    m_rgb_left_rec_pub = m_it->advertiseCamera("/left_rectified/rgb_rectified", 1);
    m_rgb_right_rec_pub = m_it->advertiseCamera("/right_rectified/rgb_rectified", 1);
    m_rgb_big_left_rec_pub = m_it->advertiseCamera("/big_left_rectified/rgb_rectified", 1);
    m_rgb_big_right_rec_pub = m_it->advertiseCamera("/big_right_rectified/rgb_rectified", 1);
    m_Q_pub = m_nh.advertise<std_msgs::Float64MultiArray>("/bumblebee/q_matrix", 1);
    getInitParams();
    // reading intrinsic parameters
    FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", intrinsic_filename);
        exit(-1);
    }

    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;
    //1024 x 768
    fs["BM1"] >> BM1;
    fs["BD1"] >> BD1;
    fs["BM2"] >> BM2;
    fs["BD2"] >> BD2;

    fs.open(extrinsic_filename, CV_STORAGE_READ);
    // reading extrinsic parameters
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsic_filename);
        exit(-1);
    }

    fs["R"] >> R;
    fs["T"] >> T;
    //1024 x 768
    fs["BR"] >> BR;
    fs["BT"] >> BT;

    std::cout << "rotation matrix:" <<std::endl<< BR << std::endl;
    std::cout << "translation matrix:" <<std::endl<< BT << std::endl;

    img_size.width = 640;
    img_size.height = 480;
    bimg_size.width = 1024;
    bimg_size.height = 768;
    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
    stereoRectify( BM1, BD1, BM2, BD2, bimg_size, BR, BT, BR1, BR2, BP1, BP2, BQ, CALIB_ZERO_DISPARITY, -1, bimg_size, &broi1, &broi2 );
    //stereoRectify( M1, ldMat, M2, rdMat, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
    std::cout << P1 << std::endl;
    std::cout << P2 << std::endl;
    printf("1024 x 768 p matrix\n");
    std::cout << BP1 << std::endl;
    std::cout << BP2 << std::endl;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
    initUndistortRectifyMap(BM1, BD1, BR1, BP1, bimg_size, CV_16SC2, bmap11, bmap12);
    initUndistortRectifyMap(BM2, BD2, BR2, BP2, bimg_size, CV_16SC2, bmap21, bmap22);
    initCameraInfo();
    //initTransform2Base(0.27, 0, 0.5, 0, 0.244346, 0);
    initTransform2Base(0, 0, 0, -1.5707963, 0, -1.5707963);
}

void stereoCam::initTransform2Base(float x, float y, float z, float roll, float pitch, float yaw)
{
  btMatrix3x3 rotMat;
  rotMat.setEulerYPR(yaw, pitch, roll);
  m_tr = tf::Transform(rotMat, btVector3(x,y,z));
}

void stereoCam::initCameraInfo()
{
  m_left_cam_info.height = 480;
  m_left_cam_info.width = 640;
  m_right_cam_info.height = 480;
  m_right_cam_info.width = 640;
  m_left_rec_cam_info.height = 480;
  m_left_rec_cam_info.width = 640;
  m_right_rec_cam_info.height = 480;
  m_right_rec_cam_info.width = 640;

  //1024 x 768
  m_big_left_rec_cam_info.height = 768;
  m_big_left_rec_cam_info.width = 1024;
  m_big_right_rec_cam_info.height = 768;
  m_big_right_rec_cam_info.width = 1024;

  m_left_cam_info.D.resize(5);
  m_right_cam_info.D.resize(5);
  for(int i = 0; i < 5; ++i)
  {
    m_left_cam_info.D[i] = D1.at<double>(0,i);
    m_right_cam_info.D[i] = D2.at<double>(0,i);
  }
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      m_left_cam_info.K[i*3+j] = M1.at<double>(i,j);
      m_right_cam_info.K[i*3+j] = M2.at<double>(i,j);
      m_left_cam_info.R[i*3+j] = R1.at<double>(i,j);
      m_right_cam_info.R[i*3+j] = R2.at<double>(i,j);
    }
  }
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      m_left_cam_info.P[i*4+j] = P1.at<double>(i,j);
      m_right_cam_info.P[i*4+j] = P2.at<double>(i,j);
    }
  }
  m_right_cam_info.distortion_model = m_left_cam_info.distortion_model = "plumb_bob";
  m_left_cam_info.roi.x_offset = (int)roi1.x;
  m_left_cam_info.roi.y_offset = (int)roi1.y;
  m_left_cam_info.roi.width = (int)roi1.width;
  m_left_cam_info.roi.height = (int)roi1.height;

  m_right_cam_info.roi.x_offset = (int)roi2.x;
  m_right_cam_info.roi.y_offset = (int)roi2.y;
  m_right_cam_info.roi.width = (int)roi2.width;
  m_right_cam_info.roi.height = (int)roi2.height;

  //rectified cam info
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      m_left_rec_cam_info.P[i*4+j] = P1.at<double>(i,j);
      m_right_rec_cam_info.P[i*4+j] = P2.at<double>(i,j);
    }
  }
  m_right_rec_cam_info.distortion_model = m_left_rec_cam_info.distortion_model = "plumb_bob";
  m_left_rec_cam_info.roi.x_offset = (int)roi1.x;
  m_left_rec_cam_info.roi.y_offset = (int)roi1.y;
  m_left_rec_cam_info.roi.width = (int)roi1.width;
  m_left_rec_cam_info.roi.height = (int)roi1.height;

  m_right_rec_cam_info.roi.x_offset = (int)roi2.x;
  m_right_rec_cam_info.roi.y_offset = (int)roi2.y;
  m_right_rec_cam_info.roi.width = (int)roi2.width;
  m_right_rec_cam_info.roi.height = (int)roi2.height;

  //big rectified cam info
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      m_big_left_rec_cam_info.P[i*4+j] = BP1.at<double>(i,j);
      m_big_right_rec_cam_info.P[i*4+j] = BP2.at<double>(i,j);
    }
  }
  m_big_right_rec_cam_info.distortion_model = m_big_left_rec_cam_info.distortion_model = "plumb_bob";
  m_big_left_rec_cam_info.roi.x_offset = (int)broi1.x;
  m_big_left_rec_cam_info.roi.y_offset = (int)broi1.y;
  m_big_left_rec_cam_info.roi.width = (int)broi1.width;
  m_big_left_rec_cam_info.roi.height = (int)broi1.height;

  m_big_right_rec_cam_info.roi.x_offset = (int)broi2.x;
  m_big_right_rec_cam_info.roi.y_offset = (int)broi2.y;
  m_big_right_rec_cam_info.roi.width = (int)broi2.width;
  m_big_right_rec_cam_info.roi.height = (int)broi2.height;

  std::cout << Q << std::endl;
  m_Q_msg.data.resize(16);
  for(int i = 0; i < 4; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      m_Q_msg.data[i*4+j] = Q.at<double>(i,j);
    }
  }
}

/** get initial parameters (only when node starts). */
void stereoCam::getInitParams()
{
  if (!m_nh.getParam("frame_id", m_frame_id))
    {
      m_frame_id = "camera";
    }

  // resolve frame ID using tf_prefix parameter
  std::string tf_prefix = tf::getPrefixParam(m_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  m_frame_id = tf::resolve(tf_prefix, m_frame_id);

  // default camera name is base name of frame ID
  size_t basepos = m_frame_id.find_last_of("/");
  m_camera_name = m_frame_id.substr(basepos+1);
  cout << m_frame_id<<endl;
  cout << m_camera_name<<endl;
}

void stereoCam::grabRGBs()
{
    extractImagesColor( &stereoCamera, DC1394_BAYER_METHOD_NEAREST, pucDeInterlacedBuffer,
            pucRGBBuffer, pucGreenBuffer, &pucRightRGB, &pucLeftRGB, &pucCenterRGB);
    m_rightRGB.data = pucRightRGB;
    m_leftRGB.data = pucLeftRGB;

    resize(m_rightRGB, m_smlRightRGB, img_size, 0, 0, INTER_LINEAR);
    resize(m_leftRGB, m_smlLeftRGB, img_size, 0, 0, INTER_LINEAR);

}

void stereoCam::publishImagePairs()
{
    m_left_cam_info.header.frame_id = m_frame_id;
    m_right_cam_info.header.frame_id = m_frame_id;
    //publish left image
    ros::Time time_now = ros::Time::now();
    m_cvi.header.stamp = time_now;
    m_cvi.header.frame_id = m_frame_id;
    m_cvi.encoding = "rgb8";
    m_cvi.image = m_smlLeftRGB;
    m_cvi.toImageMsg(m_left_msg);
    m_left_cam_info.header.stamp = time_now;
    m_rgb_left_pub.publish(m_left_msg, m_left_cam_info);

    //publish right image
    m_cvi.header.stamp = time_now;
    m_cvi.header.frame_id = m_frame_id;
    m_cvi.encoding = "rgb8";
    m_cvi.image = m_smlRightRGB;
    m_cvi.toImageMsg(m_right_msg);
    m_right_cam_info.header.stamp = time_now;
    m_rgb_right_pub.publish(m_right_msg, m_right_cam_info);

    //do rectification

    remap(m_smlLeftRGB, m_leftRec, map11, map12, INTER_LINEAR);
    remap(m_smlRightRGB, m_rightRec, map21, map22, INTER_LINEAR);

    remap(m_leftRGB, m_bigLeftRec, bmap11, bmap12, INTER_LINEAR);
    remap(m_rightRGB, m_bigRightRec, bmap21, bmap22, INTER_LINEAR);


    m_left_rec_cam_info.header.frame_id = m_frame_id;
    m_right_rec_cam_info.header.frame_id = m_frame_id;
    
    //publish left rectified image
    time_now = ros::Time::now();
    m_cvi.header.stamp = time_now;
    m_cvi.header.frame_id = m_frame_id;
    m_cvi.encoding = "rgb8";
    m_cvi.image = m_leftRec;
    m_cvi.toImageMsg(m_left_rec_msg);
    m_left_rec_cam_info.header.stamp = time_now;
    m_rgb_left_rec_pub.publish(m_left_rec_msg, m_left_rec_cam_info);
    
    //publish right rectified image
    m_cvi.header.stamp = time_now;
    m_cvi.header.frame_id = m_frame_id;
    m_cvi.encoding = "rgb8";
    m_cvi.image = m_rightRec;
    m_cvi.toImageMsg(m_right_rec_msg);
    m_right_rec_cam_info.header.stamp = time_now;
    m_rgb_right_rec_pub.publish(m_right_rec_msg, m_right_rec_cam_info);

    
    m_big_left_rec_cam_info.header.frame_id = m_frame_id;
    m_big_right_rec_cam_info.header.frame_id = m_frame_id;

    //publish big left rectified image
    time_now = ros::Time::now();
    m_cvi.header.stamp = time_now;
    m_cvi.header.frame_id = m_frame_id;
    m_cvi.encoding = "rgb8";
    m_cvi.image = m_bigLeftRec;
    m_cvi.toImageMsg(m_big_left_rec_msg);
    m_big_left_rec_cam_info.header.stamp = time_now;
    m_rgb_big_left_rec_pub.publish(m_big_left_rec_msg, m_big_left_rec_cam_info);
    
    //publish big right rectified image
    m_cvi.header.stamp = time_now;
    m_cvi.header.frame_id = m_frame_id;
    m_cvi.encoding = "rgb8";
    m_cvi.image = m_bigRightRec;
    m_cvi.toImageMsg(m_big_right_rec_msg);
    m_big_right_rec_cam_info.header.stamp = time_now;
    m_rgb_big_right_rec_pub.publish(m_big_right_rec_msg, m_big_right_rec_cam_info);

    //publish q matrix
    m_Q_pub.publish(m_Q_msg);
    m_br.sendTransform(tf::StampedTransform(m_tr, ros::Time::now(), "base_link", m_frame_id));
}

stereoCam::~stereoCam(){
    if ( dc1394_video_set_transmission( stereoCamera.camera, DC1394_OFF ) != DC1394_SUCCESS){
        fprintf( stderr, "Couldn't stop the camera?\n" );
    }
    delete[] pucDeInterlacedBuffer;
    if ( pucRGBBuffer )
        delete[] pucRGBBuffer;
    if ( pucGreenBuffer )
        delete[] pucGreenBuffer;
    // close camera
    cleanup_and_exit( camera );
    delete m_it;
}

void stereoCam::cleanup_and_exit( dc1394camera_t* camera )
{
    dc1394_capture_stop( camera );
    dc1394_video_set_transmission( camera, DC1394_OFF );
    dc1394_camera_free( camera );
}
