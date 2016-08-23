#include <opencv2/opencv.hpp>

// #include "getLanes.hpp"
#include "computer_vision/misc.hpp"

using namespace std;
using namespace cv;

// input image size 
int image_width;
int image_height;

// Hough transform
int thres_num_points;

// clustering of lines
int thres_cluster_delta_angle;
int thres_cluster_delta_rho;

// if two lanes are parallel and of certain distance, then left and right lanes are both detected. Pick the left one
int thres_parallel_delta_angle;
int thres_parallel_delta_rho;

// if two lanes are converging. Pick the right one
int thres_converge_delta_angle;
int thres_converge_delta_rho;

// method for edge detection
int detect_method = 0;
int dilation_white_size;

// method for white pixel extraction
int extract_method;
int dilation_element;
int dilation_edge_size;


/*  
    This is the main function for lane detection. It takes an image as input and returns a vector of lines.
        Each element in the returned vector contains rho and theta of the detected lane in the ground plane.
        rho - the angle between the detected lane and the heading of the robot (i.e., the camera).
        theta - the distance from the origin (bottom left of the ground plane) to the detected lane
 */
vector<Vec2f> getLanes(cv::Mat ipm_input, bool isDebug)
{
	vector<Vec2f> clusters;

    // Verify whether input image is empty
    if (ipm_input.empty()){
        cout << endl << "Error: Input image is empty. Function getLanes(cv::Mat input) aborts." << endl << endl;
        return clusters;
    }
    
    // Verify size of input images.
    int rows = ipm_input.rows;
    int cols = ipm_input.cols;
    if ((rows != image_height) || (cols != image_width)){
		cout << "Warning: forced resizing of input images" << endl;
        Size size(image_height, image_width);    
        resize(ipm_input, ipm_input, size); //resize image
    }

	// Edge detection
    cv::Mat detection_edge = edgeDetection(ipm_input, detect_method, false);
	static cv::Mat dilated_edges;
	dilated_edges = Dilation( detection_edge, dilation_element, dilation_edge_size, isDebug, "Dilated Edges" );

	// Get white pixels
    cv::Mat white_pixels = extractWhitePixel(ipm_input, extract_method, false);

	// Dilation of the white pixels
	static cv::Mat dilated_white_pixels;
	dilated_white_pixels = Dilation( white_pixels, dilation_element, dilation_white_size, isDebug, "Dilated White Pixels" );
    
	// combine edge detection and white pixel extraction
	static cv::Mat lane_boundaries;
	bitwise_and( dilated_white_pixels, dilated_edges, lane_boundaries);	
	if (isDebug) {
        imshow("Bitwise and", lane_boundaries);
    }

	// testing curve fitting
	// fitCurve(lane_boundaries, ipm_rgb);


	// Use probabilistic Hough Line Transform to detect lines
	vector<Vec2f> lines;
    // HoughLines( filtered, lines, 1, CV_PI/180, thres_num_points );
	HoughLines( lane_boundaries, lines, 1, CV_PI/180, thres_num_points );
    
    // Result cleanning: make sure the distance rho is always positive.
    for( size_t i = 0; i < lines.size(); i++ )
    {
        if (lines[i][0] < 0)
        {
            lines[i][0] = -lines[i][0];
            lines[i][1] = CV_PI + lines[i][1];
        }
        lines[i][1] = wrapTheta(lines[i][1]);
    }
    
    // Show results before clustering
    //if (isDebug) 
	if (false)
	{
        cv::Mat ipm_duplicate = ipm_input.clone();
		for( size_t i = 0; i < lines.size(); i++ )
		{
		    float rho = lines[i][0];
		    float theta = lines[i][1];
		    
		    double a = cos(theta), b = sin(theta);
		    double x0 = a*rho, y0 = b*rho;
		    Point pt1(cvRound(x0 + 1000*(-b)),
		              cvRound(y0 + 1000*(a)));
		    Point pt2(cvRound(x0 - 1000*(-b)),
		              cvRound(y0 - 1000*(a)));
		    line( ipm_duplicate, pt1, pt2, Scalar(0,255,0), 3, 8 );
		}

        imshow("Hough Line Transform Before Clustering", ipm_duplicate);
        cout << lines.size() << " raw lines found (before clustering). " << endl;
    }
    
    
    // cluster lines into groups and take averages, in order to remove duplicate segments of the same line
    // TODO: need a robust way of distinguishing the left and right lanes
    vector<int> num_of_lines;
    
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        if (isDebug) {
            //cout << "Raw line\t" << i << ":\trho=" << rho << "\ttheta=" << theta / CV_PI * 180;
        }
        
        double a = cos(theta), b = sin(theta);
        
        bool cluster_found = false;
        
        // Match this line with existing clusters
        for (size_t j=0; j < clusters.size(); j++)
        {
            Vec2f avg_line = clusters[j] / num_of_lines[j];
            float avg_rho = avg_line[0];
            float avg_theta = avg_line[1];
            
            // clustered in the same cluster if it is close to the cluster average
            if ((abs(rho - avg_rho) < thres_cluster_delta_rho) && (abs(theta - avg_theta) / CV_PI * 180 < thres_cluster_delta_angle))
            {
                clusters[j] += lines[i];
                num_of_lines[j] ++;
                cluster_found = true;
                
                //cout << "\tcluster=" << j << endl;
                break;
            }
        }
        
        // Create a new cluster if it doesn't match with any existing clusters
        if (cluster_found)
            continue;
        else
        {
            clusters.push_back(lines[i]);
            num_of_lines.push_back(1);
            //cout << "\tCluster=" << clusters.size()-1 << endl;
        }
    }
    
    // Take averages of each cluster and show results after clustering
    for (int i=0; i<clusters.size(); i++)
    {
        clusters[i] = clusters[i] / num_of_lines[i];
        float rho = clusters[i][0];
        float theta = clusters[i][1];
        //cout << "cluster\t" << i << ":\trho=" << rho << "\ttheta=" << theta / CV_PI * 180 << endl;
        
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        line( ipm_input, pt1, pt2, Scalar(0,255,0), 3, 8 );
    }
	
    
    if (isDebug) {
        imshow("Hough Line Transform After Clustering", ipm_input);
        cout << clusters.size() << " clusters found. " << endl;
    }

	// TODO: verify clusters by taking samples on the line and check color
    
    return clusters;

}
