#include "PersonCharacteristics.h"

double PersonCharacteristics::match_threshold=0.9;

PersonCharacteristics::PersonCharacteristics()
{
	alpha = .3;
}

void
PersonCharacteristics::init( cv::Mat rgb, cv::Mat mask )
{
	hist = makeHist(rgb, mask);
}

// Updates this person with new data
void
PersonCharacteristics::update( cv::Mat image, cv::Mat mask )
{
	cv::Mat otherhist = makeHist(image, mask);
	
	// Simple LPF
	hist = (1-alpha)*hist + (alpha)*otherhist;
}

cv::Mat
makeHist( cv::Mat rgb, cv::Mat mask )
{
	cv::Mat the_hist;
	
	// Convert source to HSV
	cv::Mat hsv;
	cv::cvtColor(rgb, hsv, CV_BGR2HSV);
	
	// Use the 0th and 1st channels (hue, sat)
	int channels[] = {0, 1};
	int histSize[] = {h_bins, s_bins};
	
	// Calculate the histogram
	cv::calcHist( &hsv, 1, channels, mask,   // source, number of source arrays
	              the_hist, 2, histSize, ranges);// output array, output dimensions, range of histogram
	
	// Normalize the histogram so it sums to 1
	cv::normalize( the_hist, the_hist, 0, 1, cv::NORM_MINMAX );
	
	return the_hist;
}

float
PersonCharacteristics::compare(  const PersonCharacteristics& other )
{
	if( !(hist.rows==other.hist.rows && hist.cols==other.hist.cols) ) {
		return 0;
	}
	
	// Compare histograms with correlation
	return cv::compareHist(hist, other.hist, CV_COMP_CORREL);
}

bool
PersonCharacteristics::matches( const PersonCharacteristics& other )
{
	return this->compare(other) > match_threshold;
}

cv::Mat
PersonCharacteristics::getImage(int scale)
{
	double maxVal = 0;
	cv::minMaxLoc(hist, 0, &maxVal, 0, 0);

	cv::Mat histImg = cv::Mat::zeros(s_bins*scale, h_bins*scale, CV_8UC3);
	
	// Draw a rectangle for each bin, color corresponding to its intensity
	for( int h=0; h<h_bins; h++ )
	{
		for( int s=0; s<s_bins; s++ )
		{
			float binVal = hist.at<float>(h, s);
			int intensity = cvRound(binVal*255/maxVal);
			
			cv::rectangle( histImg, cv::Point(h*scale, s*scale),
			               cv::Point( (h+1)*scale-1, (s+1)*scale-1 ),
			               cv::Scalar::all(intensity),
			               CV_FILLED );
		}
	}
	
	return histImg;
}


void
setHistogramParameters( int new_hbins, int new_sbins )
{
	h_bins = new_hbins;
	s_bins = new_sbins;
}

