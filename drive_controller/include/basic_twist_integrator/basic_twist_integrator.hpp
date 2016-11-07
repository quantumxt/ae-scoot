#ifndef _basic_twist_integrator_hpp
#define _basic_twist_integrator_hpp

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace basic_twist_integrator
{
class basic_twist_integrator
{
public:
	basic_twist_integrator( const ros::NodeHandle &_nh = ros::NodeHandle( ), const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ) );
	~basic_twist_integrator( );

	bool start( );
	void stop( );
	bool stat( );

private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Subscriber twist_stamped_sub;
	ros::Publisher odom_pub;
	const ros::SubscriberStatusCallback odom_callback;

	tf::TransformBroadcaster odom_broadcaster;

	double x;
	double y;
	double th;

	double x_covariance;
	double y_covariance;
	double yaw_covariance;

	bool pub_transform;
	std::string frame_id;

	void odom_cb( );
	void twist_stamped_cb( const geometry_msgs::TwistStampedPtr &msg );
};
}

#endif /* _basic_twist_integrator_hpp */
