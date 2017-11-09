using namespace std;
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <cv.h>
#include <highgui.h>

#define FLAG 1

const char *preset_file = "rinkaku_simple.png";
void printContours(vector< vector<cv::Point> > contours);
void cvProcess(cv::Mat &input, cv::Mat &BinaryImage, cv::Mat &contoursOnly, vector<vector<cv::Point> > contours,vector<cv::Vec4i> hierarchy);
void createBinaryImage(cv::Mat &input, cv::Mat &output);
void createContoursImage(cv::Mat &img, vector< vector<cv::Point> > contours,vector<cv::Vec4i> hierarchy);
void convertColorToGray(cv::Mat &input, cv::Mat &processed);

int main(int argc, char *argv[]){
  cv::Mat input, gray, binaryImage, processed, contoursOnly;
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  cvProcess(input, binaryImage, contoursOnly, contours, hierarchy);
  
  return 0;
}

void cvProcess(cv::Mat &input, cv::Mat &binaryImage, cv::Mat &contoursOnly, vector<vector<cv::Point> > contours,vector<cv::Vec4i> hierarchy){
  cv::Mat gray,processed;
  input = cv::imread(preset_file, 1);
  if(input.empty()){
    fprintf(stderr, "cannot open %s\n", preset_file);
    exit(0);
  }

  cv::resize(input, input, cv::Size(), 0.5, 0.5,CV_INTER_NN);
  convertColorToGray(input, gray);
  createBinaryImage(gray, binaryImage);

  processed = binaryImage.clone();
  cv::findContours(processed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
  /*int i,j;
  for(i=0;i<contours.size();i++){
    for(j=0;j<contours.at(i).size();j++){
      if(contours.at(i).at(j).x >0 &&contours.at(i).at(j).y >0){
	printf("(%d,%d)=%1.12lf,%12lf\n",i,j,contours.at(i).at(j).x,contours.at(i).at(j).y);
      }
    }
    }*/
  contoursOnly= input.clone();
  createContoursImage(contoursOnly, contours, hierarchy);
  
  cv::namedWindow("original image", 1);
  cv::namedWindow("BynaryImage", 1);
  cv::namedWindow("ContourImage", 1);
  
  cv::imshow("original image", input);
  cv::imshow("BynaryImage", binaryImage);
  cv::imshow("ContourImage", contoursOnly);

  printf("contours size : %ld\n",contours.size());
  printf("hierarchy size :%ld\n",hierarchy.size());
  cv::waitKey(0);
  

  cv::imwrite("processed.jpg", contoursOnly);
}

void convertColorToGray(cv::Mat &input, cv::Mat &processed){
  cv::Mat temp;
  vector<cv::Mat> planes;
  cv::cvtColor(input, temp, CV_BGR2YCrCb);
  cv::split(temp, planes);
  processed = planes[0];
}

void createBinaryImage(cv::Mat &input, cv::Mat &output){
  const double threshold =10;
  const double maxValue =255.0;
  cv::threshold(input, output, threshold, maxValue, CV_THRESH_BINARY);
  output = ~output;
}

void createContoursImage(cv::Mat &img, vector< vector<cv::Point> > contours, vector<cv::Vec4i> hierarchy){
  cv::Scalar color[3]={cv::Scalar(255,0,0),cv::Scalar(0,255,0),cv::Scalar(0,0,255)};
  
  int idx=0;
  for(;idx<contours.size();idx++){
    drawContours(img, contours,idx,color[idx%3],5,8,hierarchy,0);
  }
}


void printContours(vector< vector<cv::Point> > contours){
  int i,j;
  for(i=0;i<contours.size();i++){
    for(j=0;j<contours.at(i).size();j++){
      printf("(%d,%d)=%f,%f\n",i,j,contours.at(i).at(j).x,contours.at(i).at(j).y);
    }
  }
}
