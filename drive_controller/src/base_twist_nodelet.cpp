#include "base_twist/base_twist_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( base_twist, base_twist_nodelet, base_twist::base_twist_nodelet, nodelet::Nodelet )

namespace base_twist
{

base_twist_nodelet::base_twist_nodelet( const bool _autostart ) :
	odom( NULL ),
	autostart( _autostart )
{
}

base_twist_nodelet::~base_twist_nodelet( )
{
	delete odom;
}

void base_twist_nodelet::onInit( )
{
	odom = new base_twist( getNodeHandle( ), getPrivateNodeHandle( ) );

	if( autostart && !start( ) )
		ROS_ERROR( "Failed to start odom" );
}

bool base_twist_nodelet::start( )
{
	return ( NULL != odom ) && odom->start( );
}

void base_twist_nodelet::stop( )
{
	if( NULL != odom )
		odom->stop( );
}

bool base_twist_nodelet::stat( )
{
	return ( NULL != odom ) && odom->stat( );
}

}
