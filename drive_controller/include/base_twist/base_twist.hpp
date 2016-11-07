#ifndef _base_twist_hpp
#define _base_twist_hpp

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

namespace base_twist
{
class base_twist
{
public:
	base_twist( const ros::NodeHandle &_nh = ros::NodeHandle( ), const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ) );
	~base_twist( );

	bool start( );
	void stop( );
	bool stat( );

private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Publisher twist_stamped_pub;
	ros::Subscriber joint_state_sub;
	const ros::SubscriberStatusCallback twist_stamped_callback;

	std::string frame_id;
	std::string left_joint_name;
	std::string right_joint_name;
	double wheel_base;
	double wheel_diam;
	double wheel_diam2;

	void twist_stamped_cb( );
	void joint_state_cb( const sensor_msgs::JointStatePtr &msg );
};
}

#endif /* _base_twist_hpp */
