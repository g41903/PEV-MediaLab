#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <sstream>

using namespace cv;
using namespace std;

static const string OPENCV_WINDOW = "Image window";



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher ackermann_pub;

  
public:
  ImageConverter()
    : it_(nh_)
  {

    ackermann_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/ackermann_cmd_mux/input/default", 1000);

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/right/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/camera/right/image_rect/_color/output_video", 1);



//    namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    Mat src_gray, src_binary;
    Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    String cmd;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    Mat skel(cv_ptr->image.size(), CV_8UC1, cv::Scalar(0));
    Mat temp(cv_ptr->image.size(), CV_8UC1);
    Mat eroded;
    Mat cannyDst;
    Mat roi;
    Mat skeloton;

    double degree, distance;

    
    roi = calcEdgeAndROI(cv_ptr->image);

    cvtColor( roi, src_gray, CV_BGR2GRAY );


//    imshow("Gray", src_gray);

    threshold( src_gray, src_binary, 20, 255, THRESH_BINARY );

//    imshow("Bin", src_binary);

    

    // Canny( src_binary, cannyDst, 50, 200, 3 );

    // imshow("Canny", cannyDst);

    // bool done;    
    // do
    // {
    //   erode(src_binary, eroded, element);
    //   dilate(eroded, temp, element); // temp = open(src_binary)
    //   subtract(src_binary, temp, temp);
    //   bitwise_or(skel, temp, skel);
    //   eroded.copyTo(src_binary);
     
    //   done = (countNonZero(src_binary) == 0);
    // } while (!done);

    // thinning(src_binary, src_binary);
    Canny( src_binary, cannyDst, 50, 200, 3 );

//    imshow("Thin", cannyDst);
  
    vector<Vec2f> lines;
    HoughLines( cannyDst, lines, 1, CV_PI/180, 80 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];

        degree = theta * 180 / CV_PI;

        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));

        if (degree >= 0 && degree <= 180)
        {
            if(degree >= 0 && degree < 90)
            {
                line( roi, pt1, pt2, Scalar(0, 0, 255), 3, 8 );
            }
            else if((degree >= 90 && degree < 180))
            {
                line( roi, pt1, pt2, Scalar(0, 255, 0), 3, 8 );
            }

            distance = abs(rho);



            cmd = "steering";
            break;
        }

    }

    if(cmd == "steering")
    {
      publishSteering(degree, distance);
    }

//    imshow(OPENCV_WINDOW, roi);
//    waitKey(3);
    
  }

  void publishSteering(double degree, double distance)
  {
    
    ros::Rate loop_rate(10);
    ackermann_msgs::AckermannDriveStamped msg;

    cout<<"Distance : "<<distance<<endl;
    cout<<"Degree : "<<degree<<endl;

    msg.drive.speed = 1;

    if(degree>=20 && degree<90)
    {
      msg.drive.steering_angle = -(degree/90*0.34*2);
    }
    else if(degree>=90 && degree<=160)
    {
      msg.drive.steering_angle = ((180-degree)/90*0.34*2);
    }
    else
    {
      msg.drive.steering_angle = 0;
    }

    if(distance >= 260)
    {
      msg.drive.steering_angle = -2;
    }
    else if(distance <= 10)
    {
      msg.drive.steering_angle = 2;
    }
    



    ackermann_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  Mat calcEdgeAndROI(Mat gray)
  {
    int rows = gray.rows;
    int cols = gray.cols;
//    cout<<rows<<" ROWS "<<endl;
//    cout<<cols<<" COLS "<<endl;

    // for(int i=rows/3; i< rows; i++)
    // {
    //   for(int j=0; j< cols/2; j++)
    //   {
    //     gray.at<uchar>(i, j) = 0;
    //   }
    // }

    Rect rec(cols/2, 0, cols/2, rows);
    Mat image_roi = gray(rec);

    return image_roi;
  }

  void thinningIteration(cv::Mat& img, int iter)
  {
    // CV_Assert(img.channels() == 1);
    CV_Assert(img.depth() != sizeof(uchar));
    CV_Assert(img.rows > 3 && img.cols > 3);

    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

    int nRows = img.rows;
    int nCols = img.cols;

    if (img.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    int x, y;
    uchar *pAbove;
    uchar *pCurr;
    uchar *pBelow;
    uchar *nw, *no, *ne;    // north (pAbove)
    uchar *we, *me, *ea;
    uchar *sw, *so, *se;    // south (pBelow)

    uchar *pDst;

    // initialize row pointers
    pAbove = NULL;
    pCurr  = img.ptr<uchar>(0);
    pBelow = img.ptr<uchar>(1);

    for (y = 1; y < img.rows-1; ++y) {
        // shift the rows up by one
        pAbove = pCurr;
        pCurr  = pBelow;
        pBelow = img.ptr<uchar>(y+1);

        pDst = marker.ptr<uchar>(y);

        // initialize col pointers
        no = &(pAbove[0]);
        ne = &(pAbove[1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pBelow[0]);
        se = &(pBelow[1]);

        for (x = 1; x < img.cols-1; ++x) {
            // shift col pointers left by one (scan left to right)
            nw = no;
            no = ne;
            ne = &(pAbove[x+1]);
            we = me;
            me = ea;
            ea = &(pCurr[x+1]);
            sw = so;
            so = se;
            se = &(pBelow[x+1]);

            int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
                     (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
                     (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                     (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
            int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
            int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
            int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                pDst[x] = 1;
        }
    }

    img &= ~marker;
  }

/**
 * Function for thinning the given binary image
 *
 * Parameters:
 *    src  The source image, binary with range = [0,255]
 *    dst  The destination image
 */
  void thinning(const cv::Mat& src, cv::Mat& dst)
  {
      dst = src.clone();
      dst /= 255;         // convert to binary image

      cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
      cv::Mat diff;

      do {
          thinningIteration(dst, 0);
          thinningIteration(dst, 1);
          cv::absdiff(dst, prev, diff);
          dst.copyTo(prev);
      } 
      while (cv::countNonZero(diff) > 0);

      dst *= 255;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  ImageConverter ic;

  ros::spin();
  return 0;
}
