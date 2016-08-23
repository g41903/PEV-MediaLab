#ifndef GETLANES_HPP_
#define GETLANES_HPP_

#include <opencv2/opencv.hpp>
#include "misc.hpp"

using namespace std;
using namespace cv;

// input image size 
extern int image_width;
extern int image_height;

// Hough transform
extern int thres_num_points;

// clustering of lines
extern int thres_cluster_delta_angle;
extern int thres_cluster_delta_rho;

// if two lanes are parallel and of certain distance, then left and right lanes are both detected. Pick the left one
extern int thres_parallel_delta_angle;
extern int thres_parallel_delta_rho;

// if two lanes are converging. Pick the right one
extern int thres_converge_delta_angle;
extern int thres_converge_delta_rho;

// method for white pixel extraction
extern int detect_method;
extern int extract_method;
extern int dilation_element, dilation_edge_size, dilation_white_size;

vector<Vec2f> getLanes(cv::Mat input, bool isDebug = 1);

#endif /* GETLANES_HPP_ */

