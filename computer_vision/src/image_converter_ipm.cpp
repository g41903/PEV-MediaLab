#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "computer_vision/ipm.hpp"
#include "computer_vision/lane_message.h"


using namespace cv;
using namespace std;

static const string OPENCV_WINDOW = "Image window";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher lane_pub;

  cv_bridge::CvImagePtr cv_ptr;
  Mat src_gray, src_binary, roi, cannyDst;

  int width, height;

public:
  ImageConverter()
    : it_(nh_)
  {

    lane_pub = nh_.advertise<computer_vision::lane_message>("/lane_detection", 2);

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/right/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/camera/right/image_rect/_color/output_video", 1);

  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
        try
        {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }
        cout << "Width : " << cv_ptr->image.size().width << endl;
        cout << "Height: " << cv_ptr->image.size().height << endl;


        Mat inputImg, inputImgGray, outputImg;


        inputImg = cv_ptr->image;
        roi = calcEdgeAndROI(inputImg);

        width = roi.size().width;
        height = roi.size().height;

        // The 4-points at the input image
        vector<Point2f> origPoints;
        origPoints.push_back( Point2f(0, height) );
        origPoints.push_back( Point2f(width, height) );
        origPoints.push_back( Point2f(width/2+30, 140) );
        origPoints.push_back( Point2f(width/2-50, 140) );

        // The 4-points correspondences in the destination image
        vector<Point2f> dstPoints;
        dstPoints.push_back( Point2f(0, height) );
        dstPoints.push_back( Point2f(width, height) );
        dstPoints.push_back( Point2f(380, 0) );
        dstPoints.push_back( Point2f(270, 0) );

        // IPM object
        IPM ipm( Size(width, height), Size(width, height), origPoints, dstPoints );



        // Color Conversion
        if(roi.channels() == 3)
           cvtColor(roi, inputImgGray, CV_BGR2GRAY);
        else
           roi.copyTo(inputImgGray);

        // Process
        clock_t begin = clock();
        ipm.applyHomography( roi, outputImg );
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000*elapsed_secs);
        ipm.drawPoints(origPoints, roi );



//        showChannels(outputImg);


        cvtColor(outputImg, src_gray, CV_BGR2GRAY);



        int maxValue = 255;
        int adaptiveMethod = cv::ADAPTIVE_THRESH_GAUSSIAN_C;
        int thresholdType = cv::THRESH_BINARY_INV;
        int blockSize = 17;
        double C = 5;
        static cv::Mat detection_edge;

        threshold( src_gray, src_binary, 40, 255, THRESH_BINARY );
//        adaptiveThreshold( src_gray, src_binary, maxValue, adaptiveMethod, thresholdType, blockSize, C);

//        src_binary = extractWhitePixel(src_gray, 2, 1);

        imshow("Bin", src_binary);

        Canny( src_binary, cannyDst, 35, 200, 3 );

        imshow("Canny", cannyDst);

        vector<Vec2f> lines;
        HoughLines( cannyDst, lines, 1, CV_PI/180, 50 );
        drawLines(outputImg, lines);

        imshow(OPENCV_WINDOW, outputImg);
        waitKey(3);
    
  }

  void drawLines(Mat roi, vector<Vec2f> lines)
  {
      double degree, distance;
      double t_degree, t_distance;
      computer_vision::lane_message lane_msg;
      Vec2f left_lane;
      Vec2f right_lane;
      vector<Vec2f> left_lines;
      vector<Vec2f> right_lines;
      float left_rho = 0;
      float left_theta = 0;
      float avg_left_rho = 0;
      float avg_left_theta = 0;
      float right_rho = 0;
      float right_theta = 0;
      float avg_right_rho = 0;
      float avg_right_theta = 0;

      for( size_t i = 0; i < lines.size(); i++ )
      {
          if(abs(lines[i][0]) < width / 3)
          {
              left_lines.push_back(lines[i]);
          }
          else if(abs(lines[i][0]) >= width / 3)
          {
              right_lines.push_back(lines[i]);
          }

      }

      for( size_t i = 0; i < left_lines.size(); i++ )
      {
            left_rho += left_lines[i][0];
            left_theta += left_lines[i][1];
      }

      for( size_t i = 0; i < right_lines.size(); i++ )
      {
            right_rho += right_lines[i][0];
            right_theta += right_lines[i][1];
      }

      cout<<"left_lines : " << left_lines.size() << endl;
      cout<<"right_lines : " << right_lines.size() << endl;

      avg_left_rho = left_rho / left_lines.size();
      avg_left_theta = left_theta / left_lines.size();
      avg_right_rho = right_rho / right_lines.size();
      avg_right_theta = right_theta / right_lines.size();



      double l_a = cos(avg_left_theta), l_b = sin(avg_left_theta);
      double l_x0 = l_a * avg_left_rho, l_y0 = l_b * avg_left_rho;
      Point avg_left_pt1(cvRound(l_x0 + 1000*(-l_b)),
                cvRound(l_y0 + 1000*(l_a)));
      Point avg_left_pt2(cvRound(l_x0 - 1000*(-l_b)),
                cvRound(l_y0 - 1000*(l_a)));

      double r_a = cos(avg_right_theta), r_b = sin(avg_right_theta);
      double r_x0 = r_a * avg_right_rho, r_y0 = r_b * avg_right_rho;
      Point avg_right_pt1(cvRound(r_x0 + 1000*(-r_b)),
                cvRound(r_y0 + 1000*(r_a)));
      Point avg_right_pt2(cvRound(r_x0 - 1000*(-r_b)),
                cvRound(r_y0 - 1000*(r_a)));

      line( roi, Point(width / 3 , 0), Point(width / 3 , height), Scalar(255,255,0), 3, 8 );
      line( roi, avg_left_pt1, avg_left_pt2, Scalar(0,255,0), 3, 8 );
      line( roi, avg_right_pt1, avg_right_pt2, Scalar(255,0,0), 3, 8 );

      degree = avg_right_rho * 180 / CV_PI;
      distance = abs(avg_right_theta);

//      if ((degree >= 0 && degree <= 80) || (degree >= 100 && degree <= 180))
//      {
//          line( roi, avg_pt1, avg_pt2, Scalar(0,255,0), 3, 8 );
//          lane_msg.flag_detected = 1;
//          lane_msg.rho = distance;
//          lane_msg.theta = degree;
//          lane_pub.publish(lane_msg);
//      }

//      if(lines.size() == 0)
//      {
//          lane_msg.flag_detected = 0;
//          lane_msg.rho = -1;
//          lane_msg.theta = 0;
//          lane_pub.publish(lane_msg);
//      }

  }

  Mat calcEdgeAndROI(Mat gray)
  {
    int rows = gray.rows;
    int cols = gray.cols;

    Rect rec(0, rows/2, cols, rows/2);
    Mat image_roi = gray(rec);

    return image_roi;
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
  cv::Mat edgeDetection(cv::Mat rgb_frame, bool debug_mode)
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


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_ipm");

  ImageConverter ic;

  ros::spin();
  return 0;
}
