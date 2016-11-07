#include "base_twist/base_twist.hpp"

#include <ros/ros.h>

namespace base_twist
{
base_twist::base_twist( const ros::NodeHandle &_nh, const ros::NodeHandle &_nh_priv ) :
	nh( _nh ),
	nh_priv( _nh_priv ),
	twist_stamped_callback( boost::bind(&base_twist::twist_stamped_cb, this) )
{
	nh_priv.param( "wheel_base", wheel_base, 0.2635 );
	nh_priv.param( "wheel_diam", wheel_diam, 0.0750 );
	nh_priv.param( "wheel_diam2", wheel_diam2, wheel_diam );
	nh_priv.param<std::string>( "frame_id", frame_id, "base_link" );
	nh_priv.param<std::string>( "left_wheel_joint", left_joint_name, "left_wheel_joint" );
        nh_priv.param<std::string>( "right_wheel_joint", right_joint_name, "right_wheel_joint" );
}

base_twist::~base_twist( )
{
}

bool base_twist::start( )
{
	if( !( twist_stamped_pub = nh.advertise<geometry_msgs::TwistStamped>( "robot_twist", 1, twist_stamped_callback, twist_stamped_callback, ros::VoidConstPtr( ), false ) ) )
		return false;

	twist_stamped_cb( );

	return true;
}

void base_twist::stop( )
{
	if( joint_state_sub )
		joint_state_sub.shutdown( );

	if( twist_stamped_pub )
		twist_stamped_pub.shutdown( );
}

bool base_twist::stat( )
{
	return twist_stamped_pub;
}

void base_twist::twist_stamped_cb( )
{
	if( twist_stamped_pub.getNumSubscribers( ) > 0 )
	{
		if( !joint_state_sub && !( joint_state_sub = nh.subscribe( "joint_state", 1, &base_twist::joint_state_cb, this ) ) )
			ROS_ERROR( "Failed to start joint state subscription" );
	}
	else if( joint_state_sub )
		joint_state_sub.shutdown( );
}

void base_twist::joint_state_cb( const sensor_msgs::JointStatePtr &msg )
{
	unsigned int i;
 
	bool joint_recv = false;
 
	double rad_right = 0;
	double rad_left = 0;

	for( i = 0; i < msg->name.size(); i++)
	{
		if(msg->name[i] == right_joint_name)
		{
			rad_right = msg->velocity[i];
			joint_recv = true;
		}
		else if(msg->name[i] == left_joint_name)
		{
			rad_left = msg->velocity[i];
			joint_recv = true;
		}
	}

	if( !joint_recv )
		return;

	geometry_msgs::TwistStamped twist_stamped;
	twist_stamped.header = msg->header;
	twist_stamped.header.frame_id = frame_id;

	//generate twist/
	twist_stamped.twist.linear.x = ( rad_right * ( wheel_diam / 4.0 ) )  + ( rad_left * ( wheel_diam2  / 4.0 ) );
	twist_stamped.twist.angular.z = ( rad_right * (wheel_diam / ( 2.0 * wheel_base ) ) ) - ( rad_left * ( wheel_diam2 / ( 2.0 * wheel_base ) ) );

	//publish the message
	twist_stamped_pub.publish(twist_stamped);

}
}
