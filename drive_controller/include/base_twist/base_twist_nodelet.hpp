#ifndef _base_twist_nodelet_hpp
#define _base_twist_nodelet_hpp

#include "base_twist/base_twist.hpp"

#include <nodelet/nodelet.h>

namespace base_twist
{
	class base_twist_nodelet : public nodelet::Nodelet
	{
	public:
		base_twist_nodelet( const bool _autostart = true );
		~base_twist_nodelet( );
		bool start( );
		void stop( );
		bool stat( );

	private:
		virtual void onInit( );
		base_twist *odom;
		const bool autostart;
	};
}

#endif /* _base_twist_nodelet_hpp */
