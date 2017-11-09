//findContours

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <cv.h>
#include <highgui.h>

int main(){
  cv::Mat input, input_gray, binaryImage;

  input =cv::imread("rinkaku_sample.png");
  //cv::cvtColor(input, input, CV_BGR2GRAY);
  const double threshold =100.0;
  const double maxValue =255.0;
  //cv::threshold(input, binaryImage, threshold, maxValue, cv::THRESH_BINARY);

  binaryImage = ~binaryImage;
  //cv::vector< cv::vector<cv::point> > contours;
  //cv::vector<cv::Vec4i> hierarchy;
  //cv::findContours(binayImage, contours, hierarchy, CV_RETER_EXTERNAL, CV_CHAIN_APPROX_S);
  cv::namedWindow("original image", 1);
  cv::imshow("original image", input);
  
  cv::waitKey(0);

  return 0;
}
