#ifndef PERSON_CHARACTERISTICS_H
#define PERSON_CHARACTERISTICS_H

#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include <string>

class PersonCal
{
	public:
		PersonCal();
		PersonCal(const PersonCal& other); // copy constructor
		PersonCal( cv::Mat rgb, cv::Mat mask );// Constructor takes an RGB image and a mask
		void init  ( cv::Mat image, cv::Mat mask ); // Initializes this person
		void update( cv::Mat image, cv::Mat mask ); // Updates this person with new data
		
		// Returns a metric (0-1) of how well this person matches another
		float compare( const PersonCal& other );
		bool  matches( const PersonCal& other );
		
		// Returns an image representing the histogram.  Each box is scale*scale pixels
		cv::Mat getImage();
		
		// Set static histogram parameters
		static void setHistogramParameters( int new_hbins, int new_sbins );
	
		static double getAlpha(){ return alpha; }
		static void   setAlpha(double newalpha){
			CV_Assert(0<alpha && 1>alpha);
			alpha = newalpha;
		}
		
		static double getMatchThresh(){ return match_threshold; }
		static void   setMatchThresh(double newthresh){
			CV_Assert(0<newthresh && 1>newthresh);
			match_threshold = newthresh;
		}
	
		// Assignment operator
		PersonCal& operator = (const PersonCal& rhs);
		
	private:
		// Histogram in hue-saturation space
		cv::Mat hist;
		
		// True if "init" or constructor taking arguments was called
		// Must be true to use update, compare, matches, or getImage
		bool    initialized;
		
		/* Static member variables and functions */
		static cv::Mat makeHist( cv::Mat image, cv::Mat mask );
		
		// Histogram size
		static int h_bins;
		static int s_bins;
		static int hist_scale;
		
		// Low-pass filter coeficient used when updating
		static double alpha;
		
		// Threshold for histogram similarity
		static double match_threshold;
		
		// Ranges for hue, saturation
		static float h_ranges[];
		static float s_ranges[];
		static const float* ranges[];
};

#endif // PERSON_CHARACTERISTICS_H
