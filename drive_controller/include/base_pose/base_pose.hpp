#ifndef _base_pose_hpp
#define _base_pose_hpp

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace base_pose
{
class base_pose
{
public:
	base_pose( const ros::NodeHandle &_nh = ros::NodeHandle( ), const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ) );
	~base_pose( );

	bool start( );
	void stop( );
	bool stat( );

private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Publisher odom_pub;
	ros::Subscriber joint_state_sub;
	const ros::SubscriberStatusCallback odom_callback;

	bool pub_transform;
	std::string frame_id;
	std::string child_frame_id;
	std::string left_joint_name;
	std::string right_joint_name;
	
	double wheel_base;
	double wheel_diam;
	double wheel_diam2;

	double x_covariance;
	double y_covariance;
	double yaw_covariance;

	tf::TransformBroadcaster odom_broadcaster;

	void odom_cb( );
	void joint_state_cb( const sensor_msgs::JointStatePtr &msg );
};
}

#endif /* _base_pose_hpp */
