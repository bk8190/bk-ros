#include <person_tracker/PTSounds.h>

namespace PTSounds {

PTSoundPlayer::PTSoundPlayer(ros::NodeHandle parent, string name):
	state_        (state_none),
	prev_state_   (state_none),
	nh_           (parent, name),
	sound_client_ (nh_, "/robotsound")
{
	nh_.param<string>("sound_directory", directory_, "");
	
	ROS_INFO("[PTSounds] Looking for mp3 files in \"%s\"", directory_.c_str());
}

void PTSoundPlayer::setState( State new_state )
{
	if( new_state == state_none )
	{
		ROS_ERROR("[PTSounds] Tried to set an invalid state");
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
				ROS_ERROR("[PTSounds] Bad PTSoundPlayer state");
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
				ROS_ERROR("[PTSounds] Bad PTSoundPlayer state");
			break;
		}
	}
	
	prev_state_ = state_;
}

void PTSoundPlayer::playInitializationSound()
{
	ROS_INFO("[PTSounds] [Played] Initialization");
	sound_client_.playWave(directory_ + "Hello_where_are_you.ogg");
}

void PTSoundPlayer::playLockOnSound()
{

	ROS_INFO("[PTSounds] [Played] Lock On");
	int sound_num = rand() % 2;
	switch(sound_num)
	{
		case 0:
			sound_client_.playWave(directory_ + "There_you_are.ogg");
			break;
	
		case 1:
			sound_client_.playWave(directory_ + "There_you_are2.ogg");
			break;
			
		default:
			ROS_ERROR("[PTSounds] Tried to play bad sound");
			break;
	}
}

void PTSoundPlayer::playLockedOnSound()
{
	ROS_INFO("[PTSounds] [Played] Locked On");
	int sound_num = rand() % 3;
	switch(sound_num)
	{
		case 0:
			sound_client_.playWave(directory_ + "I_see_you.ogg");
			break;
	
		case 1:
			sound_client_.playWave(directory_ + "There_you_are.ogg");
			break;
			
		case 2:
			sound_client_.playWave(directory_ + "There_you_are2.ogg");
			break;
			
		default:
			ROS_ERROR("[PTSounds] Tried to play bad sound");
			break;
	}
}

void PTSoundPlayer::playTargetLostSound()
{
	ROS_INFO("[PTSounds] [Played] Target Lost");
	int sound_num = rand() % 2;
	switch(sound_num)
	{
		case 0:
			sound_client_.playWave(directory_ + "Target_Lost.ogg");
			break;
			
		case 1:
			sound_client_.playWave(directory_ + "Sentry_Mode_Activated.ogg");
			break;
			
		default:
			ROS_ERROR("[PTSounds] Tried to play bad sound");
			break;
	}
}

void PTSoundPlayer::playSearchingSound()
{
	ROS_INFO("[PTSounds] [Played] Searching sound");
	int sound_num = rand() % 4;
	switch(sound_num)
	{
		case 0:
			sound_client_.playWave(directory_ + "Searching.ogg");
			break;
			
		case 1:
			sound_client_.playWave(directory_ + "Is_Anyone_There.ogg");
			break;
			
		case 2:
			sound_client_.playWave(directory_ + "Still_there.ogg");
			break;
			
		case 3:
			sound_client_.playWave(directory_ + "Canvasing.ogg");
			break;
			
		default:
			ROS_ERROR("[PTSounds] Tried to play bad sound");
			break;
	}
}

};//namespace
