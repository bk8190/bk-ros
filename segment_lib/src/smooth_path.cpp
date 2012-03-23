#include <segment_lib/segment_lib.h>
namespace segment_lib{

// Combines some segments (ex. if there is a turn followed by an arc, replaces it by a single arc)
p_nav::Path combineSegments(const p_nav::Path& path)
{
	p_nav::Path newpath;
	newpath = replaceMultipleTurns(path);
	//newpath = replaceTurnArcs(path);
	return newpath;
}


// Performs multiple passes of resampling
p_nav::Path
smoothPathMultiple(const p_nav::Path& path, int passes)
{
	p_nav::Path oldpath, newpath;
	newpath = path;
	
	for( int i=0; i<passes; i++ ){
		oldpath = newpath;
		newpath = smoothPath(oldpath);
	}
	
	return newpath;
}


geometry_msgs::Quaternion averageAngles(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
	double angle = 0.5*(tf::getYaw(q1) + tf::getYaw(q2));
	return tf::createQuaternionMsgFromYaw( angle );
}

geometry_msgs::Quaternion subtractAngles(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
	double angle = tf::getYaw(q1) - tf::getYaw(q2);
	return tf::createQuaternionMsgFromYaw( angle );
}


// Resamples the path's segments
p_nav::Path
smoothPath(const p_nav::Path& path)
{
	// Smoothing is possible with more than 2 segments
	if( path.segs.size() < 2 ) {
		return path;
	}
	
	p_nav::Path         smoothpath;
	p_nav::PathSegment  newseg, currseg, nextseg, prevseg;
	geometry_msgs::Pose start, start2, end, end2;

	// Preserve the path's header information
	smoothpath.header = path.header;
	smoothpath.segs.clear();
	
	for( unsigned int i=0; i<path.segs.size() - 1; i++ )
	{
		currseg = path.segs.at(i);
		
		// Skip over turn-in-place segments
		if( currseg.seg_type != p_nav::PathSegment::ARC ) {
			smoothpath.segs.push_back(currseg);
			continue;
		}
		
		// First segment in the path:
		if( i==0 )
		{
			nextseg = path.segs.at(i+1);
			
			start   = getStartPose(currseg).pose;
			end     = getEndPose  (currseg).pose;
			end2    = getStartPose(nextseg).pose;
			
			end.orientation = averageAngles(end.orientation, end2.orientation);
		}
		// Last segment in the path:
		else if( i == path.segs.size()-1 )
		{
			prevseg = path.segs.at(i-1);
		
			start   = getEndPose  (prevseg).pose;
			start2  = getStartPose(currseg).pose;
			end     = getEndPose  (currseg).pose;
			
			start.orientation = averageAngles(start.orientation, start2.orientation);
		}
		// Middle segment:
		else
		{
			prevseg = path.segs.at(i-1);
			nextseg = path.segs.at(i+1);
			
			start   = getEndPose  (prevseg).pose;
			start2  = getStartPose(currseg).pose;
			end     = getEndPose  (currseg).pose;
			end2    = getStartPose(nextseg).pose;
			
			start.orientation = averageAngles(start.orientation, start2.orientation);
			end.orientation   = averageAngles(end  .orientation, end2  .orientation);
		}

		// Create a new segment
		newseg = makePathSegment(start.position.x, start.position.y, tf::getYaw(start.orientation),
				                     end  .position.x, end  .position.y, tf::getYaw(end  .orientation));

		// Preserve some information from the current segment
		newseg.header     = currseg.header;
		newseg.seg_number = currseg.seg_number;
		
		// Make sure the smoothing didn't mess anything up too badly
		double start_ang_diff = tf::getYaw(subtractAngles(getStartPose(currseg).pose.orientation, getStartPose(newseg).pose.orientation));
		double end_ang_diff = tf::getYaw(subtractAngles(getEndPose(currseg).pose.orientation, getEndPose(newseg).pose.orientation));
		double angle_threshold = 3.141/16;
		
		double length_difference = (newseg.seg_length - currseg.seg_length)/currseg.seg_length;
		double length_threshold = .10;
		
		if( fabs(length_difference) > length_threshold
		 || fabs(start_ang_diff   ) > angle_threshold
		 || fabs(end_ang_diff     ) > angle_threshold )
		{
			ROS_WARN("Smoothing produced weird results at segment %lu/%lu", i+1, path.segs.size());
			ROS_WARN("length_difference=%.2f, angle diffs %.2fpi, %.2fpi", length_difference, start_ang_diff/3.141, end_ang_diff/3.141);
			smoothpath.segs.push_back(currseg);
		}
		else {		
			smoothpath.segs.push_back(newseg);
		}
	}
	return smoothpath;
}


// Combines some segments (ex. if there is a turn followed by an arc, replaces it by a single arc)
p_nav::Path replaceTurnArcs(const p_nav::Path& path)
{
	const double max_combine_angle = pi/4;
	p_nav::PathSegment currseg, nextseg, newseg;
	geometry_msgs::Pose                    start1, start2, end1, end2;
	p_nav::Path        newpath;
	newpath.header = path.header;
	double dtheta;
	bool combined_segs; // whether or not we combined a segment on this loop
	
	for(unsigned int path_idx = 0; path_idx < path.segs.size(); path_idx++)
	{
		currseg = path.segs.at(path_idx);
		combined_segs = false;
		
		// If there is a segment left after this one
		if( path_idx < path.segs.size() - 1 )
		{
			nextseg    = path.segs.at(path_idx+1);
		
			// If current is arc and next is turn (or other way around)
			if(((currseg.seg_type == p_nav::PathSegment::SPIN_IN_PLACE)
				  && (nextseg.seg_type == p_nav::PathSegment::ARC)) ||
				 ((currseg.seg_type == p_nav::PathSegment::ARC)
				  && (nextseg.seg_type == p_nav::PathSegment::SPIN_IN_PLACE)))
			{
				//ROS_INFO("Detected arc/turn or turn/arc");
				// If both are moving in the same direction (curvature has same sign)
				if( currseg.curvature * nextseg.curvature > 0.0 )
				{
					
					if( currseg.seg_type == p_nav::PathSegment::ARC)
						dtheta = currseg.seg_length;
					else
						dtheta = nextseg.seg_length;
						
					//ROS_INFO("Same curvature. Dtheta = %.2f",dtheta);
					
					if( fabs(dtheta) < max_combine_angle)
					{
						ROS_INFO("Combining segments %d and %d", path_idx, path_idx+1);
						
						start1 = interpSegment(currseg, 1, .1).front().pose;
						//end1   = interpSegment(currseg, 1, .1).back().pose;
						//start2 = interpSegment(nextseg   , 1, .1).front().pose;
						end2   = interpSegment(nextseg   , 1, .1).back().pose;
					
						// Combine both into a single segment
						newseg = makePathSegment(start1.position.x, start1.position.y, tf::getYaw(start1.orientation),
								                     end2.position.x  , end2.position.y  , tf::getYaw(end2.orientation));
						newseg.header     = currseg.header;
						newseg.seg_number = currseg.seg_number;
				
						// Push back the new segment.  Increment the loop index so we skip the next segment.
						newpath.segs.push_back(newseg);
						combined_segs = true;
						path_idx++;
					}// max combine angle
				}// same curvature
			}// arc+turn
		}// >1 seg left
		
		if( !combined_segs ) {
			newpath.segs.push_back(currseg);
		}
	}
	
	return newpath;
}
	

// Combines all consecutive turn-in-place segments
p_nav::Path replaceMultipleTurns(const p_nav::Path& path)
{
	p_nav::PathSegment currseg, nextseg, newseg;
	int end_idx;
	geometry_msgs::PoseStamped start, end;
	p_nav::Path newpath;
	newpath.header = path.header;
	
	for(unsigned int path_idx = 0; path_idx < path.segs.size(); path_idx++)
	{
		currseg = path.segs.at(path_idx);
		
		// If the current seg is a turn in place
		if(currseg.seg_type == p_nav::PathSegment::SPIN_IN_PLACE)
		{
			end_idx = path_idx;
			
			// Look forward and find the last consecutive turn-in-place segs
			while( end_idx+1 < path.segs.size() // the next seg is in range and the next seg is a spin
			    && path.segs.at(end_idx+1).seg_type == p_nav::PathSegment::SPIN_IN_PLACE)
			{
				end_idx++;
				//ROS_INFO("Combining seg %d with %d", path_idx, end_idx);
			}
			
			start = getStartPose(currseg);
			end   = getEndPose  (path.segs.at(end_idx));

			// Combine all into a single segment
			newseg = makePathSegment(
			           start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation),
					       end.pose.position.x  , end.pose.position.y  , tf::getYaw(end.pose.orientation));
			newseg.header     = currseg.header;
			newseg.seg_number = currseg.seg_number;
			newpath.segs.push_back(newseg);
			
			// We've now taken care of all segments including end_idx
			path_idx = end_idx;
			if( path_idx >= path.segs.size() )
				break;
		}// if turn in place
		else{
			newpath.segs.push_back(currseg);
		}
	}//for
	return newpath;
}

};//namespace
