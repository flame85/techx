#include <sys/time.h>
#include "stereocam.h"
#include "my_util.h"
#define SAVE_IMG 0
int main(int argc, char* argv[]) 
{
/*#if SAVE_IMG
  if (argc < 2)
  {
    printf("in saving mode. please give directory to save images\n");
    return -1;
  }
#endif*/
  char* intrinsic_filename = 0;
  char* extrinsic_filename = 0;
  if(argc > 1)
  {    
    for( int i = 1; i < argc; i++ )
    {
      if( strcmp(argv[i], "-i" ) == 0 )
          intrinsic_filename = argv[++i];
      else if( strcmp(argv[i], "-e" ) == 0 )
          extrinsic_filename = argv[++i];
      else
      {
          printf("Command-line parameter error: unknown option %s\n", argv[i]);
          return -1;
      }
    }
  }
  else
  {
    intrinsic_filename = (char*)"intrinsics.yml";
    extrinsic_filename = (char*)"extrinsics.yml";
  }
  if(intrinsic_filename == 0)
  {
    printf("error: intrinsic  parameter file not exist. use -i [intrinsic_filename]\n");
    return -1;
  }
  if(extrinsic_filename == 0)
  {
    printf("error: extrinsic  parameter file not exist. use -e [extrinsic_filename]\n");
    return -1;
  }
    ros::init(argc, argv, "Bumblebee");
    //cv::namedWindow("rightBGR", CV_WINDOW_NORMAL);
    //cv::namedWindow("leftBGR", CV_WINDOW_NORMAL);
    stereoCam cam(intrinsic_filename, extrinsic_filename);
    //Mat leftBGR, rightBGR;
    //ros::Rate loop_rate(5);
#if(SAVE_IMG)
    int imgCounter = 0;
#else
    int pubCounter = 0;
    double start = getSecofNow();
#endif
    while(ros::ok())
    {
        cam.grabRGBs();
#if(SAVE_IMG)
        cvtColor(cam.m_smlRightRGB, cam.m_smlRightRGB, CV_RGB2BGR);
        cvtColor(cam.m_smlLeftRGB, cam.m_smlLeftRGB, CV_RGB2BGR);
        cvtColor(cam.m_rightRGB, cam.m_rightRGB, CV_RGB2BGR);
        cvtColor(cam.m_leftRGB, cam.m_leftRGB, CV_RGB2BGR);
        imshow("rightBGR", cam.m_smlRightRGB);
        imshow("leftBGR", cam.m_smlLeftRGB);
        char key = (char)waitKey(10);
        if(key == 'q' || key == 27)
        {
          printf("quit grabbing\n");
          break;
        }
        if(key == 's')
        {
          char sleft_filename[256];
          sprintf(sleft_filename,"/home/mou/Downloads/calibration_imgs/small_left%03d.tif", imgCounter);
          char sright_filename[256];
          sprintf(sright_filename,"/home/mou/Downloads/calibration_imgs/small_right%03d.tif", imgCounter);;
          imwrite(sleft_filename,cam.m_smlLeftRGB);
          printf("saving img left to %s\n", sleft_filename);
          imwrite(sright_filename,cam.m_smlRightRGB);
          printf("saving img right to %s\n", sright_filename);

          char left_filename[256];
          sprintf(left_filename,"/home/mou/Downloads/calibration_imgs/left%03d.tif", imgCounter);
          char right_filename[256];
          sprintf(right_filename,"/home/mou/Downloads/calibration_imgs/right%03d.tif", imgCounter);;
          imwrite(left_filename,cam.m_leftRGB);
          printf("saving img left to %s\n", left_filename);
          imwrite(right_filename,cam.m_rightRGB);
          printf("saving img right to %s\n", right_filename);
          imgCounter++;
        }
        if(key == 'r')
        {
          imgCounter--;
          printf("press 's' to retake frame %d\n", imgCounter);
        }
#else
        cam.publishImagePairs();
        pubCounter++;
        if(pubCounter == 50)
        {
          printf("publishing images in %lf Hz\n", (double)pubCounter / (getSecofNow()-start));
          pubCounter = 0;
          start = getSecofNow();
        }
        ros::spinOnce();
        //loop_rate.sleep();
#endif
    }
    return 0;   
}
