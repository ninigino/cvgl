#include <opencv2/opencv.hpp>
#include <stdio.h>

#define FLAG 0

char *preset_file = "fruits.jpg";

void convertColorToGray(cv::Mat &input, cv::Mat &processed);
void func_sobel(cv::Mat &input, cv::Mat &processed);

int main(int argc, char *argv[]){
  char *input_file;

  cv::Mat input, gray, processed;

  if(argc == 2){
    input_file = argv[1];
  }
  else{
    input_file = preset_file;
  }

  input = cv::imread(input_file, 1);
  if(input.empty()){
    fprintf(stderr, "cannot open %s\n", input_file);
    exit(0);
  }

  //convertColorToGray(input, gray);
  func_sobel(input, processed); 

  cv::namedWindow("original image", 1);
  cv::namedWindow("processed image", 1);

  cv::imshow("original image", input);
  cv::imshow("processed image", processed);

  cv::waitKey(0);

  cv::imwrite("processed.jpg", processed);

  return 0;
}

void convertColorToGray(cv::Mat &input, cv::Mat &processed){

#if FLAGS //use built-in function

  cv::Mat temp;
  std::vector<cv::Mat> planes;
  cv::cvtColor(input, temp, cv_BGR2YCrCb);
  cv::split(temp, planes);
  processed = planes[0];

#else

  cv::Size s = input.size();
  processed.create(s, CV_8UC1);

  for(int j=0; j<s.height; j++){
    uchar *ptr1, *ptr2;
    ptr1 = input.ptr<uchar>(j);
    ptr2 = processed.ptr<uchar>(j);

    for(int i=0; i<s.width; i++){
      double y = 0.114 * ((double)ptr1[0] + 0.587 * (double)ptr1[1] + 0.299 * (double)ptr1[2]);

      if(y > 255) y = 255;
      if(y < 0) y = 0;

      *ptr2 = (uchar)y;
      ptr1 += 3;
      ptr2++;
    }
  }
#endif
}

void func_sobel(cv::Mat &input, cv::Mat &processed){
  cv::Mat temp;
  cv::Sobel(input, temp, CV_8U, 1, 1);
  processed = temp;
}
