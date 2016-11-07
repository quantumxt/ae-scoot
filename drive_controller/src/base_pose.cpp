#include "base_pose/base_pose.hpp"

#include <ros/ros.h>

namespace base_pose
{
base_pose::base_pose( const ros::NodeHandle &_nh, const ros::NodeHandle &_nh_priv ) :
	nh( _nh ),
	nh_priv( _nh_priv ),
	odom_callback( boost::bind(&base_pose::odom_cb, this) ),
	x_covariance( 20 ),
	y_covariance( 20 ),
	yaw_covariance( 50 )
{
	nh_priv.param( "wheel_base", wheel_base, 0.4500 );
	nh_priv.param( "wheel_diam", wheel_diam, 0.1800 );
	nh_priv.param( "wheel_diam2", wheel_diam2, wheel_diam );
	nh_priv.param<std::string>( "frame_id", frame_id, "odom" );
	nh_priv.param<std::string>( "child_frame_id", child_frame_id, "base_link" );
	nh_priv.param<std::string>( "left_wheel_joint", left_joint_name, "left_wheel_joint" );
	nh_priv.param<std::string>( "right_wheel_joint", right_joint_name, "right_wheel_joint" );
	nh_priv.param( "pub_transform", pub_transform, true );
	nh_priv.param( "x_covariance", x_covariance, x_covariance);
	nh_priv.param( "y_covariance", y_covariance, y_covariance);
	nh_priv.param( "yaw_covariance", yaw_covariance, yaw_covariance );
}

base_pose::~base_pose( )
{
}

bool base_pose::start( )
{
	if( !( odom_pub = nh.advertise<nav_msgs::Odometry>( "odom", 1, odom_callback, odom_callback, ros::VoidConstPtr( ), false ) ) )
		return false;

	odom_cb( );

	return true;
}

void base_pose::stop( )
{
	if( joint_state_sub )
		joint_state_sub.shutdown( );

	if( odom_pub )
		odom_pub.shutdown( );
}

bool base_pose::stat( )
{
	return odom_pub;
}

void base_pose::odom_cb( )
{
	if( odom_pub.getNumSubscribers( ) > 0 || pub_transform )
	{
		if( !joint_state_sub && !( joint_state_sub = nh.subscribe( "joint_state", 1, &base_pose::joint_state_cb, this ) ) )
			ROS_ERROR( "Failed to start joint state subscription" );
	}
	else if( joint_state_sub )
		joint_state_sub.shutdown( );
}

void base_pose::joint_state_cb( const sensor_msgs::JointStatePtr &msg )
{
	static ros::Time last_time = msg->header.stamp;
	static double x = 0;
	static double y = 0;
	static double th = 0;

	static double old_left = 0;
	static double old_right = 0;

	double left_diff = 0;
	double right_diff = 0;
	double dt = 0;

	unsigned int i;

	bool joint_recv = false;

	double rad_right = 0;
	double rad_left = 0;

	double vel_right = 0;
	double vel_left = 0;

	for( i = 0; i < msg->name.size(); i++)
	{
		if(msg->name[i] == right_joint_name)
		{
			rad_right = msg->position[i];
			vel_right = msg->velocity[i];
			joint_recv = true;
		}
		else if(msg->name[i] == left_joint_name)
		{
			rad_left = msg->position[i];
			vel_left = msg->velocity[i];
			joint_recv = true;
		}
	}

	if( !joint_recv )
		return;

	nav_msgs::Odometry odom;
	odom.header = msg->header;
	odom.header.frame_id = frame_id;
	odom.child_frame_id = child_frame_id;

	right_diff = rad_right - old_right;
	left_diff = rad_left - old_left;

	dt = (odom.header.stamp - last_time).toSec();

	//generate pose
	x += ( ( ( wheel_diam  * left_diff ) / 4.0 ) + ( ( wheel_diam2 * right_diff ) / 4.0 ) ) * cos( th );
	y += ( ( ( wheel_diam  * left_diff ) / 4.0 ) + ( ( wheel_diam2 * right_diff ) / 4.0 ) ) * sin( th );
	th += ( ( wheel_diam  * right_diff ) / ( 2.0 * wheel_base ) ) - ( ( wheel_diam * left_diff ) / ( 2.0 * wheel_base ) );

	//generate twist
	odom.twist.twist.linear.x = ( ( ( wheel_diam * vel_left ) / 4.0 ) + ( ( wheel_diam2 * vel_right ) / 4.0 ) ) * cos( th );
	odom.twist.twist.linear.y = ( ( ( wheel_diam * vel_left ) / 4.0 ) + ( ( wheel_diam2 * vel_right ) / 4.0 ) ) * sin( th );
	odom.twist.twist.angular.z = ( ( wheel_diam * vel_right ) / ( 2.0 * wheel_base ) ) - ( ( wheel_diam * vel_left ) / ( 2.0 * wheel_base ) );

	//set covariances
	odom.pose.covariance[0] = x_covariance;
	odom.pose.covariance[7] = y_covariance;
	odom.pose.covariance[14] = FLT_MAX;
	odom.pose.covariance[21] = FLT_MAX;
	odom.pose.covariance[28] = FLT_MAX;
	odom.pose.covariance[35] = yaw_covariance;

	odom.twist.covariance[0] = .1;
	odom.twist.covariance[7] = .1;
	odom.twist.covariance[14] = 1000000000;
	odom.twist.covariance[21] = 1000000000;
	odom.twist.covariance[28] = 1000000000;
	odom.twist.covariance[35] = .1;

	//publish the message
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;

	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

	//publish the message
        odom_pub.publish(odom);

        if( pub_transform )
        {
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header = msg->header;
                odom_trans.header.frame_id = frame_id;
                odom_trans.child_frame_id = child_frame_id;

                odom_trans.transform.translation.x = odom.pose.pose.position.x;
                odom_trans.transform.translation.y = odom.pose.pose.position.y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom.pose.pose.orientation;

                //send the transform
                odom_broadcaster.sendTransform(odom_trans);
        }

	odom_pub.publish(odom);

	old_left = rad_left;
	old_right = rad_right;

	last_time = odom.header.stamp;
}
}
