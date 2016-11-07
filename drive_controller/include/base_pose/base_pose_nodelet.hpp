#ifndef _base_pose_nodelet_hpp
#define _base_pose_nodelet_hpp

#include "base_pose/base_pose.hpp"

#include <nodelet/nodelet.h>

namespace base_pose
{
	class base_pose_nodelet : public nodelet::Nodelet
	{
	public:
		base_pose_nodelet( const bool _autostart = true );
		~base_pose_nodelet( );
		bool start( );
		void stop( );
		bool stat( );

	private:
		virtual void onInit( );
		base_pose *odom;
		const bool autostart;
	};
}

#endif /* _base_pose_nodelet_hpp */
