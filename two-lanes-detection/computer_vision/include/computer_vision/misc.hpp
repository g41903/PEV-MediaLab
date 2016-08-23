#ifndef MISC_HPP_
#define MISC_HPP_
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define HSV 0
#define HLS 1
#define ADAPTIVE 2

// parameters for white pixel extraction
extern int hueMinValue, hueMaxValue;
extern int satMinValue, satMaxValue;
extern int volMinValue, volMaxValue;
extern int lightMinValue, lightMaxValue;

// extraction of white pixels
extern float thres_white_init;
extern int thres_exposure_max;
extern int thres_exposure_min;

// This function takes an angle in the range [-3*pi, 3*pi] and
// wraps it to the range [-pi, pi].
float wrapTheta( float theta );

// Construct a new image using only one single channel of the input image
// if color_image is set to 1, create a color image; otherwise a single-channel image is returned.
// 0 - B; 1 - G; 2 - R
cv::Mat getSingleChannel(cv::Mat input, int channel, bool color_image);

// Show R,G,B channels of an image
void showChannels(cv::Mat input);

// Extract white pixels from a RGB image 
cv::Mat extractWhitePixel(cv::Mat rgb_frame, int extract_method = ADAPTIVE, bool debug_mode = false);

// Edge detection
cv::Mat edgeDetection(cv::Mat rgb_frame, int detect_method, bool debug_mode = false);

// @function Dilation 
cv::Mat Dilation( cv::Mat src, int dilation_elem, int dilation_size, bool debug_mode = false, string title = "" );

void fitCurve(cv::Mat lane_boundaries, cv::Mat ipm_rgb);

// Fit a second-order polynomial to a vector of points using Least Squares
// More details at http://www.personal.psu.edu/jhm/f90/lectures/lsq2.html
// and http://vilipetek.com/2013/10/07/polynomial-fitting-in-c-using-boost/
std::vector<double> polyFit(const std::vector<Point> pts);

#endif
