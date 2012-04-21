#include "PersonCharacteristics.h"

// Static member initialization
double PersonCharacteristics::match_threshold = 0.9;
double PersonCharacteristics::alpha      = 0.3;
int    PersonCharacteristics::h_bins     = 30;
int    PersonCharacteristics::s_bins     = 32;
int    PersonCharacteristics::hist_scale = 10;

float  PersonCharacteristics::h_ranges[] = {0, 180};
float  PersonCharacteristics::s_ranges[] = {0, 256};
const float* PersonCharacteristics::ranges[] = {h_ranges, s_ranges};


// Default constructor - init() MUST be called before this object is used
PersonCharacteristics::PersonCharacteristics():
	hist(),
	initialized(false)
{}


// Constructor takes an RGB image and a mask
PersonCharacteristics::PersonCharacteristics( cv::Mat rgb, cv::Mat mask )
{
	init(rgb, mask);
}


// copy constructor
PersonCharacteristics::PersonCharacteristics(const PersonCharacteristics& other):
	hist( other.hist.clone() ),
	initialized( other.initialized)
{}


void PersonCharacteristics::init( cv::Mat rgb, cv::Mat mask )
{
	hist = makeHist(rgb, mask);
	initialized = true;
}


// Updates this person with new data
void PersonCharacteristics::update( cv::Mat image, cv::Mat mask )
{
	CV_Assert(initialized);
	
	cv::Mat otherhist = makeHist(image, mask);
	
	// Simple LPF
	hist = (1-alpha)*hist + (alpha)*otherhist;
}


cv::Mat PersonCharacteristics::makeHist( cv::Mat rgb, cv::Mat mask )
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
	
	the_hist.at<float>(0,0) = 0;
	
	// Normalize the histogram so it sums to 1
	cv::normalize( the_hist, the_hist, 0, 1, cv::NORM_MINMAX );
	
	return the_hist;
}


float PersonCharacteristics::compare(  const PersonCharacteristics& other )
{
	CV_Assert( this->initialized && other.initialized );
	CV_Assert( hist.rows==other.hist.rows && hist.cols==other.hist.cols );
	
	// Compare histograms with correlation
	return cv::compareHist(hist, other.hist, CV_COMP_CORREL);
}


bool PersonCharacteristics::matches( const PersonCharacteristics& other )
{
	return this->compare(other) > match_threshold;
}


cv::Mat PersonCharacteristics::getImage()
{
	CV_Assert(initialized);
	
	double maxVal = 0;
	cv::minMaxLoc(hist, 0, &maxVal, 0, 0);

	cv::Mat histImg = cv::Mat::zeros(s_bins*hist_scale, h_bins*hist_scale, CV_8UC3);
	
	// Draw a rectangle for each bin, color corresponding to its intensity
	for( int h=0; h<h_bins; h++ )
	{
		for( int s=0; s<s_bins; s++ )
		{
			float binVal = hist.at<float>(h, s);
			int intensity = cvRound(binVal*255/maxVal);
			
			cv::rectangle( histImg, cv::Point(h*hist_scale, s*hist_scale),
			               cv::Point( (h+1)*hist_scale-1, (s+1)*hist_scale-1 ),
			               cv::Scalar::all(intensity),
			               CV_FILLED );
		}
	}
	
	return histImg;
}


void PersonCharacteristics::setHistogramParameters( int new_hbins, int new_sbins )
{
	h_bins = new_hbins;
	s_bins = new_sbins;
}
