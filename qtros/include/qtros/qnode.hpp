/**
 * @file /include/qtros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtros_QNODE_HPP_
#define qtros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QMatrix4x4>
#include <nav_msgs/Odometry.h>
#include "LinearMath/btTransform.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/


/*****************************************************************************
** Class
*****************************************************************************/

typedef btQuaternion Quaternion;
typedef btTransform Transform;
typedef btVector3 Vector3;
class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

signals:
	void loggingUpdated();
    void rosShutdown();
    void addTransform(QMatrix4x4 trans);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
        ros::Subscriber sub;
    QStringListModel logging_model;
    btScalar* gltransform; // double
    void vodomCallback(const nav_msgs::OdometryConstPtr& odoMsg);
};


#endif /* qtros_QNODE_HPP_ */
