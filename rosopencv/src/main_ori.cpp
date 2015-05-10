#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <limits>
#include <numeric>
#include <stdlib.h>
#include <string>
#include <math.h>


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

vector<Point2f> left_image; 
vector<Point2f> right_image; 
vector<Point2f> max_con;

Mat canny_output;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
Mat src_gray;
int thresh = 100;
Mat logoWarped;
int largest_area=0;
int largest_contour_index=0;
Rect bounding_rect;
double pi=3.14;
Mat red_image;

void img_proc(Mat img)
{
	Mat imageMain = img;

	left_image.push_back(Point2f(0,640)); 
	left_image.push_back(Point2f(480,640)); 
	left_image.push_back(Point2f(480,0));
	left_image.push_back(Point2f(0,0));  
	
	right_image.push_back(Point2f(0.0,480.0)); 
	right_image.push_back(Point2f(480.0,480.0)); 
	right_image.push_back(Point2f(480.0,0.0));
	right_image.push_back(Point2f(0.0,0.0));  
	
	namedWindow( "Display window", WINDOW_NORMAL );
	imshow( "Display window", imageMain );

	Mat H = findHomography( left_image,right_image,0 );
	//Mat H = getPerspectiveTransform(left_image,right_image);

	warpPerspective(imageMain,logoWarped,H,imageMain.size() );
	showFinal(logoWarped);
	
	Mat  src = logoWarped;
	
	// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );
	
	Mat thresholded_image;
	inRange(src_gray, Scalar(40, 0, 180),Scalar(135, 110, 255), red_image);
	threshold(red_image, thresholded_image,150,180,CV_THRESH_BINARY_INV);
	
  // Detect edges using canny
  Canny( thresholded_image , canny_output, thresh, thresh*2, 3 );

  // Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//cout << " Calculating Contours " <<endl;
  // Get the moments
	//cout << contours.size() <<endl;

	vector<Point2f> mc( contours.size() );
	vector<Moments> mu( contours.size() );

  for( int i = 0; i < contours.size(); i++ ) { 
		mu[i] = moments( contours[i], false ); 
	}

	//cout << " Calculating moments " <<endl;
  //  Get the mass centers:
 
  for( int i = 0; i < contours.size(); i++ ) {
		mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
	}
	//cout << " Calculating Mass center " <<endl;
  // Draw contours

  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	/* for( int i = 0; i< contours.size(); i++ )
	     {
       Scalar color = Scalar( 0, 0, 255 );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
	Scalar colors = Scalar( 255, 255, 255 );
       circle( drawing, mc[i], 4, colors, -1, 8, 0 );
     }
	*/

	//cout << " Contour circles" <<endl;
  // Show in a window

  
	//the image has the size of 259*194 which is converted to show the position of centroid in real-world room of size 10*10 ft

	//for( int i = 0; i< contours.size(); i++ )
	//{
	//printf("mass centroid %d x= %.2f ft  y= %.2f ft \n", i , mc[i].x/*0.03861*/, mc[i].y/*0.05154*/);
	//}

 
	for( int i = 0; i< contours.size(); i++ ) {// iterate through each contour.
		double a=contourArea( contours[i],false);  //  Find the area of contour
    if(a>largest_area){
			largest_area=a;
      largest_contour_index=i;                //Store the index of largest contour
      bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
    }

	}

	/*Scalar color( 255,255,255);
	drawContours( drawing, contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored 
	//index.
	rectangle(drawing, bounding_rect,  Scalar(0,255,0),1, 8,0);
	*/

	cout <<"Tot Contour: "<< contours.size()<< endl;
	cout <<"INDEX: "<< largest_contour_index << endl;

	Scalar color = Scalar( 0, 0, 255 );
	drawContours( drawing, contours,largest_contour_index, color, 2, 8, hierarchy, 0, Point() );
	Scalar colors = Scalar( 255, 255, 255 );
  circle( drawing, mc[largest_contour_index], 4, colors, -1, 8, 0 );

	namedWindow( "Contours",0 );
  imshow( "Contours", drawing );
	
	p.x = mc[largest_contour_index].x;
	p.y = mc[largest_contour_index].y;
	p.z = float (atan((p.y/p.x)*180/pi));

}

void showFinal(Mat src2)
{
	Mat gray,src2final;
	cvtColor(src2,gray,CV_BGR2GRAY);
	threshold(gray,gray,0,255,CV_THRESH_BINARY);
	src2.copyTo(src2final,gray);
	Mat finalImage = src2final;
	namedWindow( "output", WINDOW_NORMAL);
	imshow("output",finalImage);
}
 
static const char WINDOW[] = "Image Processed";
 
image_transport::Publisher pub;
geometry_msgs::Point p; 

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
    return;
  }
 
/*    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        for(int j=0; j<cv_ptr->image.cols; j++)
        {
            for(int k=0; k<cv_ptr->image.channels(); k++)
            {
                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
            }
        }
    }
*/     
	img_proc(cv_ptr->image); 
  cv::imshow(WINDOW, cv_ptr->image);
  cv::waitKey(10);
  pub.publish(cv_ptr->toImageMsg());
}



 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processor");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Point>("get_robot_feedback", 1000);

  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
  image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
  cv::destroyWindow(WINDOW);
  pub = it.advertise("usb_cam/image_processed", 1);
	while(ros::ok()){
		chatter_pub.publish(p);
		ros::Rate loop_rate(1);        
		ros::spinOnce();
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
	} 
}

