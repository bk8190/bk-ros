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
PathSegment makePathSegment(double x1, double y1, double t1, double x2, double y2, double t2)
{
	PathSegment seg;	
	double dx = x2-x1;
	double dy = y2-y1;
	double dth = t2-t1;
	double eps = .001;
	
	// No change in theta: line segment
	if(dth == 0)
	{
		seg.seg_type = PathSegment::LINE;
		seg.seg_length = sqrt(dx*dx + dy*dy);
		seg.ref_point = geometry_msgs::Point(x1,y1);
		seg.init_tan_angle = tf::M
		seg.curvature = 0.0;
	}
	// No change in position: turn in place
	else if( abs(dx)<eps && abs(dy)<eps )
	{
		seg.seg_type = PathSegment::SPIN_IN_PLACE
	
	}
	// Else: arc segment
	else
	{
		seg.seg_type = PathSegment::ARC;
	}
}
