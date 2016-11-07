#ifndef _basic_twist_integrator_nodelet_hpp
#define _basic_twist_integrator_nodelet_hpp

#include "basic_twist_integrator/basic_twist_integrator.hpp"

#include <nodelet/nodelet.h>

namespace basic_twist_integrator
{
	class basic_twist_integrator_nodelet : public nodelet::Nodelet
	{
	public:
		basic_twist_integrator_nodelet( const bool _autostart = true );
		~basic_twist_integrator_nodelet( );
		bool start( );
		void stop( );
		bool stat( );

	private:
		virtual void onInit( );
		basic_twist_integrator *odom;
		const bool autostart;
	};
}

#endif /* _basic_twist_integrator_nodelet_hpp */
