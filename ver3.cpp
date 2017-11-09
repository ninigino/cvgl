using namespace std;
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <stdlib.h>
#include <GL/glut.h>

#define WINDOW_X 500
#define WINDOW_Y 500
#define WINDOW_NAME "3Dmap" 
#define VAR_HEIGHT 10

const char *preset_file = "tk3.jpg";
cv::Mat input, binaryImage, contoursOnly;
vector<vector<cv::Point> > contours,contoursResize;
vector<cv::Vec4i> hierarchy;
vector<vector<int> > hierarchyResize;
int window_x = 500;
int window_y = 500;
double g_angle1 = 0.0;
double g_angle2 = 0.0;
double g_distance = 5.0;
double g_angle3 = 0.0;
double g_angle4 = 3.141592/4;
bool g_isLeftButtonOn = false;
bool g_isRightButtonOn = false;

void init_GL(int argc, char *argv[]);
void init();
void set_callback_functions();
void glut_display();
void glut_keyboard(unsigned char key, int x, int y);
void glut_mouse(int button, int state, int x, int y);
void glut_motion(int x, int y);

void drawPlane();
void drawContours(int n, int level);
void draw_axis();
void drawSide();
  
void cvProcess();
void createBinaryImage(cv::Mat &input, cv::Mat &output);
void createContoursImage(cv::Mat &img, vector< vector<cv::Point> > contours,vector<cv::Vec4i> hierarchy);
void convertColorToGray(cv::Mat &input, cv::Mat &processed);
void addHeightHierarchies(int contourNumber);
void resizeHierarchies();
void resizeContours();
void imageShow();

void printContours();
void printHierarchies();
void printHierarchyResize();

int main(int argc, char *argv[]){

  cvProcess();
  init_GL(argc, argv);
  init();
  set_callback_functions();
  glutMainLoop();

  return 0;
}

void drawContours(int n, int level){
  GLfloat facecolor2[] = {1.0, 1, 0, 1.0};
  GLfloat facecolor1[] = {1.0, 0.0, 0.0};
  int i;
  for(n=0;n<contoursResize.size();n++){
    glPushMatrix();
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, facecolor2);
    
    glNormal3d(0.0, 1.0, 0.0);
    glBegin(GL_LINE_LOOP);
    for(i=0;i<contoursResize[n].size();i++){
      GLdouble point[] = {(double)contoursResize[n][i].x/window_x-0.5,
                          (double)hierarchyResize[n][4]/VAR_HEIGHT,
                          (double)contoursResize[n][i].y/window_y-0.5};
      glVertex3dv(point);
    }
    glEnd();
    glPopMatrix();
  }
  return;
}

void drawSide(){
  GLfloat facecolor1[] = {0.0, 1.0, 0.0, 1.0};
  double x1,x2,y1,y2,z1,z2;
  int i,n;
  for(n=1;n<contoursResize.size();n++){
    for(i=0;i<contoursResize[n].size()-1;i++){
      x1 = (double)contoursResize[n][i].x/window_x-0.5;
      x2 = (double)contoursResize[n][i+1].x/window_x-0.5;
      y1 = (double)hierarchyResize[n][4]/VAR_HEIGHT;
      y2 = (double)(hierarchyResize[n][4]-1)/VAR_HEIGHT;
      z1 = (double)contoursResize[n][i].y/window_y-0.5;
      z2 = (double)contoursResize[n][i+1].y/window_y-0.5;
      GLdouble pointA[] = {x1,y1,z1};
      GLdouble pointB[] = {x2,y1,z2};
      GLdouble pointC[] = {x2,y2,z2};
      GLdouble pointD[] = {x1,y2,z1};
      cv::Vec3d AB(x2-x1,y1-y1,z2-z1);
      cv::Vec3d AC(x2-x1,y2-y1,z2-z1);
      cv::Vec3d normal = AC.cross(AB);

      glPushMatrix();
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, facecolor1);
      glNormal3d(-normal[0], -normal[1], -normal[2]);
      glBegin(GL_POLYGON);
      glVertex3dv(pointA);
      glVertex3dv(pointB);
      glVertex3dv(pointC);
      glVertex3dv(pointD);
      glEnd();
      glPopMatrix();
    }
  }
  return;
}


//callback function for display.
void glut_display(){
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(30.0, 1.0, 0.1, 100);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if (cos(g_angle2)>0){
    gluLookAt(g_distance * cos(g_angle2) * sin(g_angle1), 
	      g_distance * sin(g_angle2), 
	      g_distance * cos(g_angle2) * cos(g_angle1), 
	      0.0, 0.0, 0.0, 0.0, 1.0, 0.0);}
  else{
    gluLookAt(g_distance * cos(g_angle2) * sin(g_angle1),
	      g_distance * sin(g_angle2),
	      g_distance * cos(g_angle2) * cos(g_angle1),
	      0.0, 0.0, 0.0, 0.0, -1.0, 0.0);}

  GLfloat lightpos[] = {5 * cos(g_angle4) * sin(g_angle3), 
			5 * sin(g_angle4), 
			5 * cos(g_angle4) * cos(g_angle3),  
			1.0};
  
  GLfloat diffuse[] = {1.0,1.0,1.0,1.0};
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);

  draw_axis();

  drawContours(0,0);

  glPushMatrix();
  drawSide();
  glPopMatrix();

  //drawPlane();
  
  glFlush();

  glDisable(GL_LIGHT0);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  glutSwapBuffers();
}



void drawPlane(){
  const GLdouble xsize = 10.0f;
  const GLdouble zsize = 10.0f;
  const int xnum = 20;
  const int znum = 20;

  glBegin(GL_POLYGON);
  glVertex3d(xsize, -0.5 , zsize);
  glVertex3d(-xsize, -0.5 , zsize);
  glVertex3d(-xsize, -0.5 , -zsize);
  glVertex3d(xsize, -0.5 , -zsize);
  glEnd();
}


void draw_axis(){
  GLdouble pointZp[] = {0.0, 0.0, 1000};
  GLdouble pointZm[] = {0.0, 0.0, -1000};
  GLdouble pointYp[] = {0.0, 1000, 0.0};
  GLdouble pointYm[] = {0.0, -1000, 0.0};
  GLdouble pointXp[] = {1000, 0.0, 0.0};
  GLdouble pointXm[] = {-1000, 0.0, 0.0};


  glColor3d(1.0, 0.0, 0.0);
  glBegin(GL_LINE_LOOP);
  glVertex3dv(pointXp);
  glVertex3dv(pointXm);
  glEnd();

  glColor3d(0.0, 1.0, 0.0);
  glBegin(GL_LINE_LOOP);
  glVertex3dv(pointYp);
  glVertex3dv(pointYm);
  glEnd();

  glColor3d(0.0, 0.0, 1.0);
  glBegin(GL_LINE_LOOP);
  glVertex3dv(pointZp);
  glVertex3dv(pointZm);
  glEnd();

}

//do anything using opencv.
void cvProcess(){
  cv::Mat gray,processed;
  input = cv::imread(preset_file, 1);
  if(input.empty()){
    fprintf(stderr, "cannot open %s\n", preset_file);
    exit(0);
  }

  //cv::resize(input, input, cv::Size(), 0.5, 0.5,CV_INTER_NN);
  window_x = input.cols/2;
  window_y = input.rows/2;
  convertColorToGray(input, gray);
  createBinaryImage(gray, binaryImage);
  
  processed = binaryImage.clone();
  cv::findContours(processed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

  if(contours.size() % 2 != 0){
    fprintf(stderr, "any conter must be enveloped.\n");
    exit(1);
  }
  contoursOnly= input.clone();
  createContoursImage(contoursOnly, contours, hierarchy);
 
  resizeHierarchies();
  resizeContours();
  addHeightHierarchies(0);
  //  imageShow();
  
  
  printf("contours size : %ld\n",contours.size());
  printf("hierarchy size :%ld\n",hierarchy.size());
  return;
}

void imageShow(){    
  cv::namedWindow("original image", 1);
  cv::namedWindow("BynaryImage", 1);
  cv::namedWindow("ContourImage", 1);
  
  cv::imshow("original image", input);
  cv::imshow("BynaryImage", binaryImage);
  cv::imshow("ContourImage", contoursOnly);
  cv::waitKey(0);
  cv::imwrite("processed.jpg", contoursOnly);
  return;
}

//convert input image to the gray scaled image.
void convertColorToGray(cv::Mat &input, cv::Mat &processed){
  cv::Mat temp;
  vector<cv::Mat> planes;
  cv::cvtColor(input, temp, CV_BGR2YCrCb);
  cv::split(temp, planes);
  processed = planes[0];
}

//convert gray scaled image to binary image.
void createBinaryImage(cv::Mat &input, cv::Mat &output){
  const double threshold =100;
  const double maxValue =255.0;
  cv::threshold(input, output, threshold, maxValue, CV_THRESH_BINARY);
  output = ~output;
}

//color each contours.
void createContoursImage(cv::Mat &img, vector< vector<cv::Point> > contours, vector<cv::Vec4i> hierarchy){
  cv::Scalar color[3]={cv::Scalar(255,0,0),cv::Scalar(0,255,0),cv::Scalar(0,0,255)};
  int idx=0;
  for(;idx<contours.size();idx++){
    drawContours(img,contours,idx,color[idx%3],5,8,hierarchy,0);
  }
}

//add height information to hierarchyResize.
void addHeightHierarchies(int contourNumber){
  static int level = 0;
 
  printf("%d, %d\n",contourNumber, level);
  hierarchyResize[contourNumber][4] = -level;
  //fore
  if(hierarchyResize[contourNumber][0] != -1){
    addHeightHierarchies(hierarchyResize[contourNumber][0]);
  }
  //child
  if(hierarchyResize[contourNumber][2] != -1){
    level--;
    addHeightHierarchies(hierarchyResize[contourNumber][2]);
    level++;
  }
  return;
}

//to delete no use contours.  
void resizeHierarchies(){
  hierarchyResize.resize(hierarchy.size()/2);
  int i,j;
  for(i=j=0;i<hierarchy.size();i+=2,j++){
    hierarchyResize[j].resize(5);
    hierarchyResize[j][0] = (hierarchy[i][0] != -1) ? hierarchy[i][0]/2 :-1;
    hierarchyResize[j][1] = (hierarchy[i][1] != -1) ? hierarchy[i][1]/2 :-1;
    hierarchyResize[j][2] = (hierarchy[i][2] != -1 &&  hierarchy[hierarchy[i][2]][2] != -1) ? hierarchy[hierarchy[i][2]][2]/2 :-1;
    hierarchyResize[j][3] = (hierarchy[i][3] != -1) ? hierarchy[hierarchy[i][3]][3]/2 :-1;
    hierarchyResize[j][4] = -1;
  }
}

//to delete no use contours.
void resizeContours(){
  contoursResize.resize(contours.size()/2);
  int i;
  for(i=0;i<contoursResize.size();i++){
    contoursResize[i] = contours[2*i];
  }
}

//initialize for opengl.
void init_GL(int argc, char *argv[]){
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH);
  glutInitWindowSize(WINDOW_X, WINDOW_Y);
  glutCreateWindow(WINDOW_NAME);
}

//initialize for this opengl.
void init(){
  glClearColor(1.0, 1.0, 1.0, 0.0);
}

//set callback functions.
void set_callback_functions(){
  glutDisplayFunc(glut_display);
  glutKeyboardFunc(glut_keyboard);
  glutMouseFunc(glut_mouse);
  glutMotionFunc(glut_motion);
  glutPassiveMotionFunc(glut_motion);
}

//callback function for keyboard.
void glut_keyboard(unsigned char key, int x, int y){
  switch(key){

  case 'q':
  case 'Q':
  case '\033': // Escキーのこと
    exit(0);
    break;
  }

  glutPostRedisplay();
}

void glut_mouse(int button, int state, int x, int y){
  if(button == GLUT_LEFT_BUTTON){
    if(state == GLUT_UP){
      g_isLeftButtonOn = false;
    }else if(state == GLUT_DOWN){
      g_isLeftButtonOn = true;
    }
  }

  if(button == GLUT_RIGHT_BUTTON){
    if(state == GLUT_UP){
      g_isRightButtonOn = false;
    }else if(state == GLUT_DOWN){
      g_isRightButtonOn = true;
    }
  }
}

void glut_motion(int x, int y){
  static int px = -1, py = -1;
  if(g_isLeftButtonOn == true){
    if(px >= 0 && py >= 0){
      g_angle1 += (double)-(x - px)/20;
      g_angle2 += (double)(y - py)/20;
    }
    px = x;
    py = y;
  }else if(g_isRightButtonOn == true){
    if(px >= 0 && py >= 0){
      // g_distance += (double)(y - py)/20;
      g_angle3 += (double)(x - px)/20;
    }
    px = x;
    py = y;
  }else{
    px = -1;
    py = -1;
  }
  glutPostRedisplay();
}

//for debaggu show all points of contours.
void printContours(){
  int i,j;
  for(i=0;i<contours.size();i++){
    for(j=0;j<contours.at(i).size();j++){
      printf("(%d,%d)=%d,%d\n",i,j,contours.at(i).at(j).x,contours.at(i).at(j).y);
    }
  }
}

//for debaggu show all hierarchy.
void printHierarchies(){
  int i;
  for(i=0;i<hierarchy.size();i++){
    printf("contours(%d) : fore = %d, rear = %d, child = %d, parent = %d\n",
	   i,hierarchy[i][0],hierarchy[i][1],hierarchy[i][2],hierarchy[i][3]);
  }
}

void printHierarchyResize(){
  int i;
  for(i=0;i<hierarchyResize.size();i++){
    printf("contours(%d) : fore=%d, rear=%d, child=%d, parent=%d, height=%d\n",
	   i,hierarchyResize[i][0],hierarchyResize[i][1],hierarchyResize[i][2],hierarchyResize[i][3], hierarchyResize[i][4]);
  }
}
  
