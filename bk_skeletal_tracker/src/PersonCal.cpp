#include "PersonCal.h"


/* Static member functions, variables */
double PersonCal::match_threshold = 0.9;
double PersonCal::alpha      = 0.3;
int    PersonCal::h_bins     = 30;
int    PersonCal::s_bins     = 32;
int    PersonCal::hist_scale = 10;
float  PersonCal::h_ranges[] = {0, 180};
float  PersonCal::s_ranges[] = {0, 256};
const float* PersonCal::ranges[] = {h_ranges, s_ranges};

double PersonCal::getAlpha(){ return alpha; }
void   PersonCal::setAlpha(double newalpha){
	CV_Assert(0<alpha && 1>alpha);
	alpha = newalpha;
}

double PersonCal::getMatchThresh(){ return match_threshold; }
void   PersonCal::setMatchThresh(double newthresh){
	CV_Assert(0<newthresh && 1>newthresh);
	match_threshold = newthresh;
}

void PersonCal::setHistogramParameters( int new_hbins, int new_sbins )
{
	h_bins = new_hbins;
	s_bins = new_sbins;
}

/* End static member functions, variables */


// Default constructor - init() must be called before this object is used
PersonCal::PersonCal():
	hist(),
	initialized(false)
{}


// Constructor takes an RGB image and a mask
PersonCal::PersonCal( const cv::Mat& rgb, const cv::Mat& mask ) {
	init(rgb, mask);
}


// copy constructor
PersonCal::PersonCal( const PersonCal& other ):
	hist       ( other.hist.clone() ),
	initialized( other.initialized  )
{}


// Assignment operator
PersonCal& PersonCal::operator = (const PersonCal& rhs) {
	this->hist        = rhs.hist.clone();
	this->initialized = rhs.initialized;
	return *this;
}
		
		
void PersonCal::init( const cv::Mat& rgb, const cv::Mat& mask ) {
	hist = makeHist(rgb, mask);
	initialized = true;
}

// Updates this person with new data
void PersonCal::update( const cv::Mat& image, const cv::Mat& mask )
{
	CV_Assert(initialized);
	
	cv::Mat otherhist = makeHist(image, mask);
	
	// Simple LPF combining my data with the new data
	hist = (1-alpha)*hist + (alpha)*otherhist;
	
	cv::Mat masked;
	image.copyTo(masked, mask);
	cv::imshow("masked", masked);
}


cv::Mat PersonCal::makeHist( const cv::Mat& rgb, const cv::Mat& mask )
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

float PersonCal::getEMD ( PersonCal& other )
{
	CV_Assert( this->initialized && other.initialized );
	CV_Assert( hist.rows==other.hist.rows && hist.cols==other.hist.cols );
	
	cv::Mat sig1 = this->getEMDSignature();
	cv::Mat sig2 = other.getEMDSignature();
	
	/*std::cout << "EMD sig 1:" << std::endl << cv::format(sig1,"csv");
	std::cout << std::endl;
	std::cout << "EMD sig 2:" << std::endl << cv::format(sig1,"csv");*/
	
	// Compute earth mover's distance
	return cv::EMD(sig1, sig2, CV_DIST_L1);
}

// Convert the histogram into a signature for EMD matching
cv::Mat PersonCal::getEMDSignature()
{
	CV_Assert(initialized);
	cv::Mat signature( h_bins*s_bins, 3, CV_32FC1 );
	
	for( int h=0; h<h_bins; h++ )
	{
		for( int s=0; s<s_bins; s++ )
		{
			int   bin_idx = h*s_bins + s;
			float bin_val = hist.at<float>(h, s);
			
			if( bin_val <= 0 ){ bin_val = 0.001; }
			
			signature.at<float>(bin_idx, 0) = bin_val;
			signature.at<float>(bin_idx, 1) = h;
			signature.at<float>(bin_idx, 2) = s;
		}
	}
	
	return signature;
}

bool PersonCal::matches( const PersonCal& other ) {
	return (this->compare(other)) > match_threshold;
}


cv::Mat PersonCal::getHist()
{
	CV_Assert(initialized);
	return hist.clone();
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



