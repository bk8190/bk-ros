#ifndef PERSON_CHARACTERISTICS_H
#define PERSON_CHARACTERISTICS_H

#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include <string>

namespace PersonCharacteristics
{

// Hue varies from 0 to 179, Sat varies from 0 to 255
float  h_ranges[] = {0, 180};
float  s_ranges[] = {0, 256};
const float* ranges[] = {h_ranges, s_ranges};
	
// Histogram size
int h_bins=30, s_bins=32;

cv::Mat makeHist( cv::Mat image, cv::Mat mask );

class PersonCharacteristics
{
	public:
		PersonCharacteristics();
		
		// Constructor takes an RGB image and a mask
		PersonCharacteristics( cv::Mat rgb, cv::Mat mask );
		
		// Initializes this person
		void init( cv::Mat image, cv::Mat mask );
		
		// Updates this person with new data
		void update( cv::Mat image, cv::Mat mask );
		
		// Returns a metric (0-1) of how well this person matches another
		float compare( const PersonCharacteristics& other );
		
		bool matches( const PersonCharacteristics& other );
		
		// Returns an image representing the histogram.  Each box is scale*scale pixels
		cv::Mat getImage(int scale);
		
		// Set static histogram parameters
		static void setHistogramParameters( int new_hbins, int new_sbins );
	
		void setAlpha(double newalpha){ alpha = newalpha; }
		double getAlpha(){ return alpha; }
		
		
		static double match_threshold;

	private:
		// Low-pass filter coeficient used when updating
		double alpha;
		
		// Histogram in hue-saturation space
		cv::Mat hist;	
};

};//namespace PersonCharacteristics

#endif // PERSON_CHARACTERISTICS_H
