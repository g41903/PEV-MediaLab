#include <opencv2/opencv.hpp>
// #include <boost/numeric/ublas/matrix.hpp>
// #include <boost/numeric/ublas/lu.hpp>
#include "computer_vision/misc.hpp"

using namespace std;
using namespace cv;

// parameters for white pixel extraction
int hueMinValue, hueMaxValue;
int satMinValue, satMaxValue;
int volMinValue, volMaxValue;
int lightMinValue, lightMaxValue;

// extraction of white pixels
float thres_white_init;
int thres_exposure_max;
int thres_exposure_min;

// This function takes an angle in the range [-3*pi, 3*pi] and
// wraps it to the range [-pi, pi].
float wrapTheta( float theta )
{
    if (theta > CV_PI)
        return theta - 2 * CV_PI;
    else if (theta < - CV_PI)
        return theta + 2 * CV_PI;
    return theta;
}

// Construct a new image using only one single channel of the input image
// if color_image is set to 1, create a color image; otherwise a single-channel image is returned.
// 0 - B; 1 - G; 2 - R
cv::Mat getSingleChannel(cv::Mat input, int channel, bool color_image)
{
    // Split channels
    vector<cv::Mat> spl;
    split(input,spl);
    
    // Return single-channel image
    if (color_image == 0)
        return spl[channel];
    
    // Create color image
    cv::Mat output;
    cv::Mat emptyMat = Mat::zeros(Size(input.cols, input.rows), CV_8UC1);
    vector<cv::Mat> channels;
    
    for (int i=0; i<3; i++)
    {
        if (i == channel)
            channels.push_back(spl[i]);
        else
            channels.push_back(emptyMat);
        
    }
    
    merge(channels, output);
    
    return output;
    
}

// Show different channels of an image
void showChannels(cv::Mat input)
{
    imshow("B", getSingleChannel(input,0, true));//b
    imshow("G", getSingleChannel(input,1, true));//g
    imshow("R", getSingleChannel(input,2, true));//r
    
    cv::Mat greyMat;
    cv::cvtColor(input, greyMat, CV_BGR2GRAY);
    imshow("Grey", greyMat);//grey-scale
}

// Edge detection
cv::Mat edgeDetection(cv::Mat rgb_frame, int detect_method, bool debug_mode)
{
	// Get a single channel
	static cv::Mat singleChannel;
	singleChannel = getSingleChannel(rgb_frame, 0, false);

	// First apply Gaussian filtering
	static cv::Mat blurred_ipm;
	GaussianBlur(singleChannel, blurred_ipm, Size(9,9), 0, 0);
	if (false) { // (debug_mode) {
        imshow("Blurred ipm", blurred_ipm);
    }

	// Edge detection
    // adaptive thresholding outperforms canny and other filtering methods
	int maxValue = 255;
	int adaptiveMethod = cv::ADAPTIVE_THRESH_GAUSSIAN_C;
	int thresholdType = cv::THRESH_BINARY_INV;
	int blockSize = 11;
	double C = 5;
	static cv::Mat detection_edge;

    adaptiveThreshold( blurred_ipm, detection_edge, maxValue, adaptiveMethod, thresholdType, blockSize, C);

    if (debug_mode) {
        imshow("Edge detection", detection_edge);
    }
	return detection_edge;
}

// Extract white pixels from a RGB image
cv::Mat extractWhitePixel(cv::Mat rgb_frame, int extract_method, bool debug_mode)
{
	// convert from RGB to HSV representation
	if (extract_method == HSV)
	{
		cv::Mat hsv_frame;
		cv::cvtColor(rgb_frame, hsv_frame, CV_RGB2HSV);

		cv::Scalar min(hueMinValue, satMinValue, volMinValue);
		cv::Scalar max(hueMaxValue, satMaxValue, volMaxValue);
		cv::Mat threshold_frame;
		cv::inRange( hsv_frame, min, max, threshold_frame);
		return threshold_frame;
	}
	else if (extract_method == HLS)
	{	
		cv::Mat hls_frame;
		cv::cvtColor(rgb_frame, hls_frame, CV_RGB2HLS);

		cv::Scalar min(hueMinValue, lightMinValue, satMinValue);
		cv::Scalar max(hueMaxValue, lightMaxValue, satMaxValue);
		cv::Mat threshold_frame;
		cv::inRange( hls_frame, min, max, threshold_frame);
		return threshold_frame;
	}
	else if (extract_method == ADAPTIVE)
	{
		// Get a single channel
		static cv::Mat singleChannel;
		singleChannel = getSingleChannel(rgb_frame, 0, false);

		// Extraction of white pixels
		double min, max;
		static cv::Mat thresholded;
		cv::minMaxLoc(singleChannel, &min, &max);

		// adaptive thresholding
		int maxValue = 255;
		int thres_count = 0;
		float thres_adaptive = thres_white_init;
		float thres_upper_bound = 1;
		float thres_lower_bound = 0;
		//int type = cv::THRESH_TOZERO;
		int type = cv::THRESH_BINARY;

		// Binary search for the proper exposure threshold
		while (thres_count < 10)
		{
			thres_count ++;	

			int thresh = min + (max - min) * thres_adaptive;
			threshold( singleChannel, thresholded, thresh, maxValue, type);

			// Deal with over-exposure
			double s = cv::sum( thresholded )[0] / 255;
			// cout << "Exposure: " << s << endl;
		
			if (s > thres_exposure_max)
			{
				// cout << "Over-exposed. s = " << s << "\tthres_adaptive = " << thres_adaptive << endl;
				thres_lower_bound = thres_adaptive;
				thres_adaptive = (thres_upper_bound + thres_lower_bound) / 2;
			}
			else if (s < thres_exposure_min)
			{
				// cout << "Under-exposed. s = " << s << "\tthres_adaptive = " << thres_adaptive << endl;
				thres_upper_bound = thres_adaptive;
				thres_adaptive = (thres_upper_bound + thres_lower_bound) / 2;
			}
			else
			{
				// cout << "Proper-exposed. s = " << s << "\tthres_adaptive = " << thres_adaptive << endl;
				break;
			}
		}
		return thresholded;
	}
}

/** @function Dilation */
cv::Mat Dilation( cv::Mat src, int dilation_elem, int dilation_size, bool debug_mode, string title )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  cv::Mat dilation_dst;
  dilate( src, dilation_dst, element );

  // Visualization
  if (debug_mode) {
	// imshow("Before Dilation", src);
    imshow(title, dilation_dst);
  }

  return dilation_dst;
}

// TODO: curve fitting
void fitCurve(cv::Mat lane_boundaries, cv::Mat ipm_rgb)
{
	std::vector<Point> filtered_points;
	int count = 0;
	for (int y = 0; y < lane_boundaries.rows; y++)
	{
		for (int x = 0; x < lane_boundaries.cols; x++)
		{
			// cout << lane_boundaries.at<uchar>(y, x) << " ";
			// In OpenCV, x and y are inverted when trying to access an element
			if (lane_boundaries.at<uchar>(y, x) == 255)
			{
				Point pt(x, y);	
				filtered_points.push_back(pt);
			}
		}
	}

	std::vector<double> coefficients = polyFit(filtered_points);
	double c1 = coefficients[0]; 
	double c2 = coefficients[1];
	double c3 = coefficients[2];
	cout << "c1 = " << c1 << "\tc2 = " << c2 << "\tc3 = " << c3 << endl;

	// cout << "filtered_points.size() = " << filtered_points.size() << "\tapproxCurve.size() = " << approxCurve.size() << endl;
	
	std::vector<Point> testCurve;
	for (int x=0; x<lane_boundaries.cols; x++)
	{
		int appro_y = c1 + c2 * x + c3 * pow(x, 2);
		Point pt(x, appro_y);
		// cout << "appro_y = " << appro_y << endl;
		testCurve.push_back(pt);
	}
	
	Scalar color = Scalar( 255, 0, 0 );
	polylines(ipm_rgb, testCurve, false, color);
	imshow("Curve detection", ipm_rgb);

    // polylines(ipm_rgb, filtered_points, false, color);
	// imshow("check input to curve detection", ipm_rgb);
	imshow("lane_boundaries", lane_boundaries);
	
}

// Fit a second-order polynomial to a vector of points
// More details at http://www.personal.psu.edu/jhm/f90/lectures/lsq2.html
// and http://vilipetek.com/2013/10/07/polynomial-fitting-in-c-using-boost/
std::vector<double> polyFit(const std::vector<Point> pts)
{
	/*
	using namespace boost::numeric::ublas;

	int nCount = pts.size();
	cout << "nCount = " << nCount << endl;

	// Create X matrix
	matrix<double> XMatrix( nCount, 3 );
	for (int i=0; i<nCount; i++)
	{	
		for (int col=0; col<3; col++)
		{	
			XMatrix(i, col) = double(pow(pts[i].x, col));
			// cout << XMatrix(i, col) << " ";
		}
		// cout << endl;
	}

	// Create Y matrix
	matrix<double> YMatrix( nCount, 1 );
	for (int i=0; i<nCount; i++)	
	{
		YMatrix(i, 0) = double(pts[i].y);
		// cout << YMatrix(i, 0) << endl;
	}

	// Least Squares, find inverse 

	// transpose X matrix
	static matrix<double> XtMatrix( trans(XMatrix) );
	// multiply transposed X matrix with X matrix
	static matrix<double> XtXMatrix( prec_prod(XtMatrix, XMatrix) );
	// multiply transposed X matrix with Y matrix
	static matrix<double> XtYMatrix( prec_prod(XtMatrix, YMatrix) );

	for (int i=0; i<3; i++)
	{	
		for (int col=0; col<3; col++)
		{	
			cout << XtXMatrix(i, col) << " ";
		}
		cout << endl;
	}

	// lu decomposition
	permutation_matrix<int> pert(XtXMatrix.size1());
	int singular = lu_factorize(XtXMatrix, pert);
	// must be singular
	BOOST_ASSERT( singular == 0 );

	// backsubstitution
	lu_substitute(XtXMatrix, pert, XtYMatrix);

	// copy the result to coeff
	return std::vector<double>( XtYMatrix.data().begin(), XtYMatrix.data().end() );
	*/
}

