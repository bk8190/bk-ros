#ifndef PTSOUNDS_H
#define PTSOUNDS_H

#include <stdlib.h>
#include <ros/ros.h>
#include <sound_play/sound_play.h>

namespace PTSounds {	

using std::string;

enum State
{
	state_none,
	state_tracking,
	state_searching
};

class PTSoundPlayer
{
	public:	
		PTSoundPlayer(ros::NodeHandle parent, string name);
		void setState( State new_state );
		void update();
	
	private:
		void playInitializationSound();
		void playLockOnSound();
		void playLockedOnSound();
		void playTargetLostSound();
		void playSearchingSound();
	
		State state_;
		State prev_state_;
		ros::NodeHandle nh_;
		sound_play::SoundClient sound_client_;
		
		string directory_;
		ros::Time next_scheduled_time_;
};

};//namespace

#endif
