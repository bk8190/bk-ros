#include "PersonCal.h"

// Static member initialization
double PersonCal::match_threshold = 0.9;
double PersonCal::alpha      = 0.3;
int    PersonCal::h_bins     = 30;
int    PersonCal::s_bins     = 32;
int    PersonCal::hist_scale = 10;
float  PersonCal::h_ranges[] = {0, 180};
float  PersonCal::s_ranges[] = {0, 256};
const float* PersonCal::ranges[] = {h_ranges, s_ranges};


// Default constructor - init() MUST be called before this object is used
PersonCal::PersonCal():
	hist(),
	initialized(false)
{}


// Constructor takes an RGB image and a mask
PersonCal::PersonCal( cv::Mat rgb, cv::Mat mask ) {
	init(rgb, mask);
}


// copy constructor
PersonCal::PersonCal(const PersonCal& other):
	hist       ( other.hist.clone() ),
	initialized( other.initialized  )
{}


// Assignment operator
PersonCal& PersonCal::operator = (const PersonCal& rhs) {
	this->hist        = rhs.hist.clone();
	this->initialized = rhs.initialized;
}
		
		
void PersonCal::init( cv::Mat rgb, cv::Mat mask ) {
	hist = makeHist(rgb, mask);
	initialized = true;
}

// Updates this person with new data
void PersonCal::update( cv::Mat image, cv::Mat mask )
{
	CV_Assert(initialized);
	
	cv::Mat otherhist = makeHist(image, mask);
	
	// Simple LPF combining my data with the new data
	hist = (1-alpha)*hist + (alpha)*otherhist;
	
	cv::Mat masked;
	image.copyTo(masked, mask);
	cv::imshow("masked", masked);
}


cv::Mat PersonCal::makeHist( cv::Mat rgb, cv::Mat mask )
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
	
	// Gaussian blur and normalization
	cv::GaussianBlur(the_hist, the_hist, cv::Size(5,5), 2);
	cv::normalize( the_hist, the_hist, 0, 1, cv::NORM_MINMAX );
	
	return the_hist;
}


float PersonCal::compare(  const PersonCal& other )
{
	CV_Assert( this->initialized && other.initialized );
	CV_Assert( hist.rows==other.hist.rows && hist.cols==other.hist.cols );
	
	// Compare histograms with correlation
	return cv::compareHist(hist, other.hist, CV_COMP_CORREL);
}


bool PersonCal::matches( const PersonCal& other ) {
	return (this->compare(other)) > match_threshold;
}


cv::Mat PersonCal::getImage()
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
			
			int hue = cvRound( (((double)(h+1))/((double)h_bins))*(180.0) );
			int sat = cvRound( (((double)(s+1))/((double)s_bins))*(256.0) );
			cv::Scalar color(hue, sat, intensity);
			
			cv::Point p1( h   *hist_scale  ,  s   *hist_scale  );
			cv::Point p2((h+1)*hist_scale-1, (s+1)*hist_scale-1);
			
//			cv::rectangle( histImg, p1, p2, cv::Scalar::all(intensity), CV_FILLED );
			cv::rectangle( histImg, p1, p2, color, CV_FILLED );
		}
	}
	
	cv::Mat retImg;
	cv::cvtColor(histImg, retImg, CV_HSV2BGR);
	
	return retImg;
}


void PersonCal::setHistogramParameters( int new_hbins, int new_sbins )
{
	h_bins = new_hbins;
	s_bins = new_sbins;
}
