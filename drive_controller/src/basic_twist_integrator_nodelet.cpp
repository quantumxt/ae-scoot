#include "basic_twist_integrator/basic_twist_integrator_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS( basic_twist_integrator, basic_twist_integrator_nodelet, basic_twist_integrator::basic_twist_integrator_nodelet, nodelet::Nodelet )

namespace basic_twist_integrator
{

basic_twist_integrator_nodelet::basic_twist_integrator_nodelet( const bool _autostart ) :
	odom( NULL ),
	autostart( _autostart )
{
}

basic_twist_integrator_nodelet::~basic_twist_integrator_nodelet( )
{
	delete odom;
}

void basic_twist_integrator_nodelet::onInit( )
{
	odom = new basic_twist_integrator( getNodeHandle( ), getPrivateNodeHandle( ) );

	if( autostart && !start( ) )
		ROS_ERROR( "Failed to start odom" );
}

bool basic_twist_integrator_nodelet::start( )
{
	return ( NULL != odom ) && odom->start( );
}

void basic_twist_integrator_nodelet::stop( )
{
	if( NULL != odom )
		odom->stop( );
}

bool basic_twist_integrator_nodelet::stat( )
{
	return ( NULL != odom ) && odom->stat( );
}

}
