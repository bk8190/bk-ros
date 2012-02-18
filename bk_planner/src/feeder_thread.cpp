#include <bk_planner/bk_planner.h>

namespace bk_planner {

void BKPlanner::runFeederThread()
{
	long period_ms = (double)1000 * 1.0; // 1/path_feeder_frequency_;
	
	ROS_INFO("bk_planner path feeder thread started, period %ld", period_ms);
	
	while(true)
	{
	
		boost::this_thread::sleep(boost::posix_time::milliseconds(period_ms));
	}
}

};//namespace
