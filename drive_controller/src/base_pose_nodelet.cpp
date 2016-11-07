#include "base_pose/base_pose_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( base_pose, base_pose_nodelet, base_pose::base_pose_nodelet, nodelet::Nodelet )

namespace base_pose
{

base_pose_nodelet::base_pose_nodelet( const bool _autostart ) :
	odom( NULL ),
	autostart( _autostart )
{
}

base_pose_nodelet::~base_pose_nodelet( )
{
	delete odom;
}

void base_pose_nodelet::onInit( )
{
	odom = new base_pose( getNodeHandle( ), getPrivateNodeHandle( ) );

	if( autostart && !start( ) )
		ROS_ERROR( "Failed to start odom" );
}

bool base_pose_nodelet::start( )
{
	return ( NULL != odom ) && odom->start( );
}

void base_pose_nodelet::stop( )
{
	if( NULL != odom )
		odom->stop( );
}

bool base_pose_nodelet::stat( )
{
	return ( NULL != odom ) && odom->stat( );
}

}
