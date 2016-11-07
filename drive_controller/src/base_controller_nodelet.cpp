#include "base_controller/base_controller_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( base_controller, base_controller_nodelet, base_controller::base_controller_nodelet, nodelet::Nodelet )

namespace base_controller
{

base_controller_nodelet::base_controller_nodelet( const bool _autostart ) :
	controller( NULL ),
	autostart( _autostart )
{
}

base_controller_nodelet::~base_controller_nodelet( )
{
	delete controller;
}

void base_controller_nodelet::onInit( )
{
	controller = new base_controller( getNodeHandle( ), getPrivateNodeHandle( ) );

	if( autostart && !start( ) )
		ROS_ERROR( "Failed to start controller" );
}

bool base_controller_nodelet::start( )
{
	return ( NULL != controller ) && controller->start( );
}

void base_controller_nodelet::stop( )
{
	if( NULL != controller )
		controller->stop( );
}

bool base_controller_nodelet::stat( )
{
	return ( NULL != controller ) && controller->stat( );
}

}
