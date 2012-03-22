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
	
// Resamples the path's segments
p_nav::Path
smoothPath(const p_nav::Path& path)
{
	p_nav::Path         smoothpath;
	p_nav::PathSegment  newseg, currentseg, nextseg;
	std::vector<geometry_msgs::PoseStamped> interp1, interp2;
	geometry_msgs            ::Pose         start, end;

	// Preserve the path's header information
	smoothpath.header = path.header;
	smoothpath.segs.clear();
	
	// Smoothing is possible with more than 2 segments
	if( path.segs.size() >= 2 )
	{
		for( unsigned int i=0; i<path.segs.size() - 1; i++ )
		{
			currentseg = path.segs.at(i);
			
			// First segment: start of segment 0 -> end of segment 1
			if( i==0 )
			{
				interp1 = interpSegment(path.segs.at(0), .01, .01);
				interp2 = interpSegment(path.segs.at(0), .01, .01);
				start   = interp1.front().pose;
				end     = interp2.back().pose;
			}
			// Last segment: start of i -> end of segment i
			else if( i == path.segs.size()-1 )
			{
				interp1 = interpSegment(path.segs.at(i), .01, .01);
				interp2 = interpSegment(path.segs.at(i), .01, .01);
				start   = interp1.front().pose;
				end     = interp2.back().pose;
			}
			// Middle segment: front of i -> front of segment i+1
			else
			{
				interp1 = interpSegment(path.segs.at(i)  , .01, .01);
				interp2 = interpSegment(path.segs.at(i+1), .01, .01);
				start   = interp1.front().pose;
				end     = interp2.front().pose;
			}

			// Create a new segment from the start of the current segment to the start of the next segment
			newseg = makePathSegment(start.position.x, start.position.y, tf::getYaw(start.orientation),
					                     end.position.x  , end.position.y  , tf::getYaw(end.orientation));

			// Preserve some information from the current segment
			newseg.header     = currentseg.header;
			newseg.seg_number = currentseg.seg_number;
			
			smoothpath.segs.push_back(newseg);
		}
		
		// Copy over the last segment as-is
		smoothpath.segs.push_back(path.segs.back());
		
		return smoothpath;
	}
	// No smoothing is possible, return the old path
	else{
		return path;
	}
}



// Combines some segments (ex. if there is a turn followed by an arc, replaces it by a single arc)
p_nav::Path replaceTurnArcs(const p_nav::Path& path)
{
	const double max_combine_angle = pi/4;
	p_nav::PathSegment currentseg, nextseg, newseg;
	geometry_msgs::Pose                    start1, start2, end1, end2;
	p_nav::Path        newpath;
	newpath.header = path.header;
	double dtheta;
	bool combined_segs; // whether or not we combined a segment on this loop
	
	for(unsigned int path_idx = 0; path_idx < path.segs.size(); path_idx++)
	{
		currentseg = path.segs.at(path_idx);
		combined_segs = false;
		
		// If there is a segment left after this one
		if( path_idx < path.segs.size() - 1 )
		{
			nextseg    = path.segs.at(path_idx+1);
		
			// If current is arc and next is turn (or other way around)
			if(((currentseg.seg_type == p_nav::PathSegment::SPIN_IN_PLACE)
				  && (nextseg.seg_type == p_nav::PathSegment::ARC)) ||
				 ((currentseg.seg_type == p_nav::PathSegment::ARC)
				  && (nextseg.seg_type == p_nav::PathSegment::SPIN_IN_PLACE)))
			{
				//ROS_INFO("Detected arc/turn or turn/arc");
				// If both are moving in the same direction (curvature has same sign)
				if( currentseg.curvature * nextseg.curvature > 0.0 )
				{
					
					if( currentseg.seg_type == p_nav::PathSegment::ARC)
						dtheta = currentseg.seg_length;
					else
						dtheta = nextseg.seg_length;
						
					//ROS_INFO("Same curvature. Dtheta = %.2f",dtheta);
					
					if( fabs(dtheta) < max_combine_angle)
					{
						ROS_INFO("Combining segments %d and %d", path_idx, path_idx+1);
						
						start1 = interpSegment(currentseg, 1, .1).front().pose;
						//end1   = interpSegment(currentseg, 1, .1).back().pose;
						//start2 = interpSegment(nextseg   , 1, .1).front().pose;
						end2   = interpSegment(nextseg   , 1, .1).back().pose;
					
						// Combine both into a single segment
						newseg = makePathSegment(start1.position.x, start1.position.y, tf::getYaw(start1.orientation),
								                     end2.position.x  , end2.position.y  , tf::getYaw(end2.orientation));
						newseg.header     = currentseg.header;
						newseg.seg_number = currentseg.seg_number;
				
						// Push back the new segment.  Increment the loop index so we skip the next segment.
						newpath.segs.push_back(newseg);
						combined_segs = true;
						path_idx++;
					}// max combine angle
				}// same curvature
			}// arc+turn
		}// >1 seg left
		
		if( !combined_segs ) {
			newpath.segs.push_back(currentseg);
		}
	}
	
	return newpath;
}
	

// Combines all consecutive turn-in-place segments
p_nav::Path replaceMultipleTurns(const p_nav::Path& path)
{
	p_nav::PathSegment currentseg, nextseg, newseg;
	int end_idx;
	geometry_msgs::PoseStamped start, end;
	p_nav::Path newpath;
	newpath.header = path.header;
	
	for(unsigned int path_idx = 0; path_idx < path.segs.size(); path_idx++)
	{
		currentseg = path.segs.at(path_idx);
		
		// If the current seg is a turn in place
		if(currentseg.seg_type == p_nav::PathSegment::SPIN_IN_PLACE)
		{
			end_idx = path_idx;
			
			// Look forward and find the last consecutive turn-in-place segs
			while( end_idx+1 < path.segs.size() // the next seg is in range and the next seg is a spin
			    && path.segs.at(end_idx+1).seg_type == p_nav::PathSegment::SPIN_IN_PLACE)
			{
				end_idx++;
				//ROS_INFO("Combining seg %d with %d", path_idx, end_idx);
			}
			
			start = getStartPose(currentseg);
			end   = getEndPose  (path.segs.at(end_idx));

			// Combine all into a single segment
			newseg = makePathSegment(
			           start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation),
					       end.pose.position.x  , end.pose.position.y  , tf::getYaw(end.pose.orientation));
			newseg.header     = currentseg.header;
			newseg.seg_number = currentseg.seg_number;
			newpath.segs.push_back(newseg);
			
			// We've now taken care of all segments including end_idx
			path_idx = end_idx;
			if( path_idx >= path.segs.size() )
				break;
		}// if turn in place
		else{
			newpath.segs.push_back(currentseg);
		}
	}//for
	return newpath;
}

};//namespace
