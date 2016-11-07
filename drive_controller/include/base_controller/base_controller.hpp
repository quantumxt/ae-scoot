#ifndef _base_controller_hpp
#define _base_controller_hpp

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace base_controller
{
class base_controller
{
public:
	base_controller( const ros::NodeHandle &_nh = ros::NodeHandle( ), const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ) );
	~base_controller( );

	bool start( );
	void stop( );
	bool stat( );

private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Subscriber twist_sub;
	ros::Publisher joint_traj_pub;
	const ros::SubscriberStatusCallback joint_traj_callback;

	std::string frame_id;
	std::string wheel_joint_name;
	std::string steer_joint_name;
	//double wheel_diam;
	//double steer_range;

	trajectory_msgs::JointTrajectory joint_traj_template;

	void joint_traj_cb( );
	void twist_cb( const geometry_msgs::TwistPtr &msg );
};
}

#endif /* _base_controller_hpp */
