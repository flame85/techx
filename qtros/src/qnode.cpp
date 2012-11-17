/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qtros/qnode.hpp"
#include "std_msgs/Float64MultiArray.h"

static const std::string topic_name = "/stereo_odometer/odometry";

/*****************************************************************************
** Namespaces
*****************************************************************************/


/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
          gltransform = new btScalar[16];
        }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
        delete[] gltransform;
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
        printf("default init\n");
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
        sub = n.subscribe(topic_name, 1, &QNode::vodomCallback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
	//ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
        sub = n.subscribe(topic_name, 1, &QNode::vodomCallback, this);
	//start();
	return true;
}

void QNode::mat2dist(const QMatrix4x4& t, double &dist){
    dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
}
///Get euler angles from affine matrix (helper for isBigTransform)
void QNode::mat2RPY(const QMatrix4x4& t, double& roll, double& pitch, double& yaw) {
    roll = atan2(t(2,1),t(2,2));
    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
    yaw = atan2(t(1,0),t(0,0));
}

bool QNode::isBigTransform(const QMatrix4x4& t)
{
    double roll, pitch, yaw, dist;
    double lastRoll, lastPitch, lastYaw, lastDist;

    mat2RPY(t, roll,pitch,yaw);
    mat2dist(t, dist);

    mat2RPY(last_transform_, lastRoll, lastPitch, lastYaw);
    mat2dist(last_transform_, lastDist);

    roll = roll/M_PI*180;
    pitch = pitch/M_PI*180;
    yaw = yaw/M_PI*180;
    lastRoll = lastRoll/M_PI*180;
    lastPitch = lastPitch/M_PI*180;
    lastYaw = lastYaw/M_PI*180;

    double deltaRoll = fabs(roll - lastRoll);
    double deltaPitch = fabs(pitch - lastPitch);
    double deltaYaw = fabs(yaw - lastYaw);
    double deltaDist = fabs(dist - lastDist);

    double max_angle = std::max(deltaRoll,std::max(deltaPitch,deltaYaw));
    /*if(t(1,3) > 0.2)
       return false;*/

    // at least 10cm or 5deg
    return (deltaDist > 0.1 || max_angle > 5);
}
void QNode::vodomCallback(const nav_msgs::OdometryConstPtr& odoMsg)
{
   const geometry_msgs::Pose& msg = odoMsg->pose.pose;
   btTransform transf = Transform(Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), 
                                  Vector3(msg.position.x, msg.position.y, msg.position.z));
   transf.getOpenGLMatrix(gltransform);
   //printf("size of trans is %d\n", sizeof(gltransform) / sizeof(double));
   QMatrix4x4 transform(static_cast<qreal*>(gltransform));
   /*transform(0,3) = transform(3,0);
   transform(1,3) = transform(3,1);
   transform(2,3) = transform(3,2);
   transform(3,0) = 0.;
   transform(3,1) = 0.;
   transform(3,2) = 0.;*/
   transform = transform.transposed();// transform for opengl use later
   if(isBigTransform(transform))
   {
     emit addTransform(transform);
     last_transform_ = transform;
   }

   /*for(int i = 0; i < 16; i++)
   {
     printf("%lf ", gltransform[i]);
   }*/

   /*printf("qmatrix is \n");
   for(int i = 0; i < 4; ++i)
   {
     for(int j = 0; j <4; ++j)
     {
       printf("%lf ", transform(i,j));
     }
     printf("\n");
   }*/
}

void QNode::run() {
  ros::Rate r(10); // 10 hz. 
  while(ros::ok() ) {

    ros::spinOnce(); 
    r.sleep();
  }
  emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
	/*ros::Rate loop_rate(1);
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;*/
  //printf("run\n");
  //ros::spin();
	//emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	emit loggingUpdated(); // used to readjust the scrollbar
}

