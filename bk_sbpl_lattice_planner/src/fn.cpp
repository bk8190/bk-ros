void ConvertStateIDPathintoXYThetaPath(const EnvironmentNAVXYTHETALAT& env, const vector<int>& stateIDPath, vector<PathSegment>& segmentPath)
{
	// Discrete state
	int x1_c, y1_c, t1_c;
	int x2_c, y2_c, t2_c;
	
	// Continuous state
	double x1, y1, t1;
	double x2, y2, t2;

	bool ret1, ret2;
	segmentPath.clear();
	
	// Make a path segment for every change in state
	for(int path_index = 0; path_index < (int)(stateIDPath->size())-1; path_index++)
	{
		// IDs of the source/target state
		int sourceID = stateIDPath->at(path_index  );
		int targetID = stateIDPath->at(path_index+1);

		// Retrieve state information about the source and target
		env.GetCoordFromState(sourceID, x1_c, y1_c, t1_c);
		env.GetCoordFromState(targetID, x2_c, y2_c, t2_c);
		
		ret1 = env.PoseDiscToCont(x1_c, y1_c, t1_c, x1, y1, t1);		
		ret2 = env.PoseDiscToCont(x2_c, y2_c, t2_c, x2, y2, t2);
		
		if(ret1<0 || ret2<0){
			ROS_ERROR("Error converting discrete pose to continuous");
			segmentPath.clear();
			return;
		}
		
		// We now have source and destination states.  Build a path segment.
		this_seg = makeSegment(x1,y1,t1, x2,y2,t2); 
		segmentPath.push_back(this_seg);
	}
}


// Returns a path segment between two points (x,y,theta)
// Note: initializes all speed/accel limits to 0
PathSegment makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2)
{
	PathSegment seg;
	double dx = x2-x1;
	double dy = y2-y1;
	double dth = t2-t1;
	double eps = .001; // Precision for floating point comparison
	
	// Arc parameters
	double chord_length, denom, signed_r, abs_r, ang_to_center, xcenter, ycenter;
	
	// Initialize the segment, set all speeds/accels to 0
	seg.max_speeds.linear.x  = 0;
	seg.max_speeds.angular.z = 0;
	seg.min_speeds.linear.x  = 0;
	seg.min_speeds.angular.z = 0;
	seg.accel_limit          = 0;
	seg.decel_limit          = 0;
	
	// No change in theta: line segment
	if(abs(dth) < eps)
	{
		seg.seg_type       = precision_navigation_msgs::PathSegment::LINE;
		seg.seg_length     = sqrt(dx*dx + dy*dy);
		seg.ref_point.x    = x1;
		seg.ref_point.y    = y1;
		seg.ref_point.z    = 0.0;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
		seg.curvature      = 0.0;
	}
	// No change in position: turn in place
	else if( abs(dx)<eps && abs(dy)<eps )
	{
		seg.seg_type       = precision_navigation_msgs::PathSegment::SPIN_IN_PLACE
		seg.seg_length     = abs(dth);
		seg.ref_point.x    = x1;
		seg.ref_point.y    = y1;
		seg.ref_point.z    = 0.0;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
		
		// Positive curvature -> CW rotation, negative curvature -> CCW rotation
		if( dth>0 )
			seg.curvature =  1.0;
		else
			seg.curvature = -1.0;
	}
	// Else: arc segment
	else
	{
		seg.seg_type = precision_navigation_msgs::PathSegment::ARC;
		seg.init_tan_angle = tf::createQuaternionMsgFromYaw(t1);
		
		// Use the formula chord_length = 2*radius*sin(dtheta) -> radius = chord_length/(2*sin(dtheta))
		// Note that from this formula, r can be either positive or negative.
		// Positive r: segment bending to the right
		// Negative r: segment bending to the left
		chord_length = sqrt(dx*dx + dy*dy);
		denom = 2*sin(dth);
		if(denom != 0)
		{
			signed_r = chord_length / (denom);
			abs_r    = abs(signed_r); // Absolute, positive radius
			
			if(signed_r != 0)		
				seg.curvature = 1/signed_r;
			else{
				seg.curvature = 0;
				ROS_ERROR("Invalid arc segment created!");
			}
		}
		else{
			seg.curvature = 0;
			ROS_ERROR("Invalid arc segment created!");
		}
		
		// Find the angle to the circle's center from the starting point
		if( dth>0 )
			ang_to_center = t1 + PI/2; // 90 degrees to the right
		if( dth<0 )
			ang_to_center = t1 - PI/2; // 90 degrees to the left
		
		// Move to the circle's center
		xcenter = x1 + abs_r*cos(ang_to_center);
		ycenter = y1 + abs_r*sin(ang_to_center);
		
		seg.ref_point.x = xcenter;
		seg.ref_point.y = ycenter;
		seg.ref_point.z = 0.0;
		
		// Arc length = r*theta
		seg.seg_length = abs_r * dth;
	}
}
