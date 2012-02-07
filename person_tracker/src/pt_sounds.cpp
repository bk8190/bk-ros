#include <stdlib.h>
#include <ros/ros.h>
#include <sound_play/sound_play.h>

namespace PTSounds
{	
	
enum State
{
	state_none,
	state_tracking,
	state_searching
};

class PTSoundPlayer
{
	public:	
		PTSoundPlayer(ros::NodeHandle nh);
		void setState( State new_state );
		void update();
	
	private:
		void playInitializationSound();
		void playLockOnSound();
		void playLockedOnSound();
		void playTargetLostSound();
		void playSearchingSound();
	
		sound_play::SoundClient sound_client_;
		State state_;
		State prev_state_;
		ros::Time next_scheduled_time_;
};

PTSoundPlayer::PTSoundPlayer(ros::NodeHandle nh):
	sound_client_(nh, "/robotsound")
{
	state_      = state_none;
	prev_state_ = state_none;
}

void PTSoundPlayer::setState( State new_state )
{
	if( new_state == state_none )
	{
		ROS_ERROR("Tried to set an invalid state");
		return;
	}
	
	this->state_ = new_state;
}

void PTSoundPlayer::update()
{
	// Play the initialization sound and set the state to "searching"
	if( state_ == state_none )
	{
		state_      = state_searching;
		prev_state_ = state_searching;
		playInitializationSound();
		next_scheduled_time_ = ros::Time::now() + ros::Duration(10);
	}

	// Check if the state changed (locked on or lost the target)
	else if( state_ != prev_state_ )
	{
		switch( state_ )
		{
			case state_tracking:
				playLockOnSound();
				next_scheduled_time_ = ros::Time::now() + ros::Duration(10);
				break;
				
			case state_searching:
				playTargetLostSound();
				next_scheduled_time_ = ros::Time::now() + ros::Duration(10);
				break;
			
			default:
				ROS_ERROR("Bad PTSoundPlayer state");
			break;
		}
	}
	
	// Check if it's been a long time in the current state
	else if( ros::Time::now() > next_scheduled_time_ )
	{
		switch( state_ )
		{
			case state_tracking:
				playLockedOnSound();
				next_scheduled_time_ = ros::Time::now() + ros::Duration(60);
				break;
				
			case state_searching:
				playSearchingSound();
				next_scheduled_time_ = ros::Time::now() + ros::Duration(10);
				break;
				
			default:
				ROS_ERROR("Bad PTSoundPlayer state");
			break;
		}
	}
	
	prev_state_ = state_;
}

void PTSoundPlayer::playInitializationSound()
{
	ROS_INFO("Initialization sound");
	sound_client_.playWave("/home/bill/dev/portal_sounds/Hello_where_are_you.ogg");
}

void PTSoundPlayer::playLockOnSound()
{

	ROS_INFO("Lock On sound");
	int sound_num = rand() % 2;
	switch(sound_num)
	{
		case 0:
			sound_client_.playWave("/home/bill/dev/portal_sounds/There_you_are.ogg");
			break;
	
		case 1:
			sound_client_.playWave("/home/bill/dev/portal_sounds/There_you_are2.ogg");
			break;
			
		default:
			ROS_ERROR("Tried to play bad sound");
			break;
	}
}

void PTSoundPlayer::playLockedOnSound()
{
	ROS_INFO("Locked On sound");
	int sound_num = rand() % 3;
	switch(sound_num)
	{
		case 0:
			sound_client_.playWave("/home/bill/dev/portal_sounds/I_see_you.ogg");
			break;
	
		case 1:
			sound_client_.playWave("/home/bill/dev/portal_sounds/There_you_are.ogg");
			break;
			
		case 2:
			sound_client_.playWave("/home/bill/dev/portal_sounds/There_you_are2.ogg");
			break;
			
		default:
			ROS_ERROR("Tried to play bad sound");
			break;
	}
}

void PTSoundPlayer::playTargetLostSound()
{
	ROS_INFO("Target Lost sound");
	int sound_num = rand() % 2;
	switch(sound_num)
	{
		case 0:
			sound_client_.playWave("/home/bill/dev/portal_sounds/Target_Lost.ogg");
			break;
			
		case 1:
			sound_client_.playWave("/home/bill/dev/portal_sounds/Sentry_Mode_Activated.ogg");
			break;
			
		default:
			ROS_ERROR("Tried to play bad sound");
			break;
	}
}

void PTSoundPlayer::playSearchingSound()
{
	ROS_INFO("Searching sound");
	int sound_num = rand() % 4;
	switch(sound_num)
	{
		case 0:
			sound_client_.playWave("/home/bill/dev/portal_sounds/Searching.ogg");
			break;
			
		case 1:
			sound_client_.playWave("/home/bill/dev/portal_sounds/Is_Anyone_There.ogg");
			break;
			
		case 2:
			sound_client_.playWave("/home/bill/dev/portal_sounds/Still_there.ogg");
			break;
			
		case 3:
			sound_client_.playWave("/home/bill/dev/portal_sounds/Canvasing.ogg");
			break;
			
		default:
			ROS_ERROR("Tried to play bad sound");
			break;
	}
}

}//Namespace
