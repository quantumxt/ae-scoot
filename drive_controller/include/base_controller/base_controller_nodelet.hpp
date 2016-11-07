#ifndef _base_controller_nodelet_hpp
#define _base_controller_nodelet_hpp

#include "base_controller/base_controller.hpp"

#include <nodelet/nodelet.h>

namespace base_controller
{
	class base_controller_nodelet : public nodelet::Nodelet
	{
	public:
		base_controller_nodelet( const bool _autostart = true );
		~base_controller_nodelet( );
		bool start( );
		void stop( );
		bool stat( );

	private:
		virtual void onInit( );
		base_controller *controller;
		const bool autostart;
	};
}

#endif /* _base_controller_nodelet_hpp */
