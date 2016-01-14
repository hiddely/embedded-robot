#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string CORNERS_WINDOW = "Edit window";

RNG rng(12345);
int thresh = 150;
int max_thresh = 255;

void detectLines(vector<Vec4i> lines);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_repub", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

   // Canny algorithm
    Mat contours;
    Canny(image,contours,50,350);
    Mat contoursInv;
    threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

    std::vector<Vec2f> lines;
    if (houghVote < 1 or lines.size() > 2){ // we lost all lines. reset 
        houghVote = 200; 
    }
    else{ houghVote += 25;} 
    while(lines.size() < 5 && houghVote > 0){
        HoughLines(contours,lines,1,PI/180, houghVote);
        houghVote -= 5;
    }
    std::cout << houghVote << "\n";
    Mat result(contours.rows,contours.cols,CV_8U,Scalar(255));
    image.copyTo(result);

  /// Detect edges using canny
  Canny( cv_ptr->image, canny_output, thresh, thresh*2, 3 );
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, drawing);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

void detectLines(vector<Vec4i> lines) {
  
}

int main(int argc, char** argv)
{
ROS_INFO("Hello22 %s", "World");
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
printf("MAIN");
ROS_INFO("Hello %s", "World");
  return 0;
}

