#include <iostream>
#include <GL/glut.h>

//----------- 各種外部変数  -------------//
//頂点座標
double vdata[4][3]={{0.2, 0, 0},  {0.4, 0.4, 0},  {0.2, 0.2, 0},  {0, 0.3, 0}};
//テセレータオブジェクト
GLUtesselator *tess;


//---------- プロトタイプ宣言 -----------//
void MakeTess();
void display();
void reshape(int w, int h);
void TessErr(GLenum error_code);
void DRAW_XYZ();


//----------- OpenGLの初期設定 -------------------------//
void GLUT_INIT()
{
  glutInitDisplayMode(GLUT_RGBA| GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("Tessellation ");
}

void GLUT_CALL_FUNC()
{
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
}

void MY_INIT()
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
  MakeTess();

  glEnable(GL_DEPTH_TEST);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void MakeTess()
{
  tess = gluNewTess();
  if(tess == NULL){
    std::cerr << "Can't make tessellator\n";
    exit(1);
  }
  gluTessCallback(tess, GLU_TESS_BEGIN, (void (*)(void))glBegin);
  gluTessCallback(tess, GLU_TESS_END, (void (*)(void))glEnd);
  gluTessCallback(tess, GLU_TESS_ERROR, (void (*)(void))TessErr);
  gluTessCallback(tess, GLU_TESS_VERTEX, (void (*)(void))glVertex3dv);
}


//----------- メイン関数 ----------------//
int main(int argc, char **argv)
{
  glutInit(&argc,argv);
  GLUT_INIT();
  GLUT_CALL_FUNC();
  MY_INIT();
  glutMainLoop();

  return 0;
}


//--------------- ここからコールバック ----------------------//
void display()
{

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
  DRAW_XYZ();  //XYZ軸の描画

  glColor3f(1,0,1);
  gluTessBeginPolygon(tess, 0);
  gluTessBeginContour(tess);
  gluTessVertex(tess, vdata[0], vdata[0]);
  gluTessVertex(tess, vdata[1], vdata[1]);
  gluTessVertex(tess, vdata[2], vdata[2]);
  gluTessVertex(tess, vdata[3], vdata[3]);
  gluTessEndContour(tess);
  gluTessEndPolygon(tess);


  glColor3f(1,1,1);
  glutSwapBuffers();  //ウィンドウに出力


}


void reshape(int w, int h)
{
  glViewport(0, 0, w, h);  //ビューポートの設定

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(30.0, (double)w / (double)h, 1.0, 100.0); //視野の設定
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0.5, 1.0, 1.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0); //視点の設定
}


void TessErr(GLenum error_code)
{
  std::cout << gluErrorString(error_code) << "\n";
  exit(1);
}

void DRAW_XYZ()
{
  glBegin(GL_LINES);

  glColor3d(0,1,0);//x
  glVertex2d(-100,0);
  glVertex2d(100, 0);

  glColor3d(1,0,0);//y
  glVertex2d(0,0);
  glVertex2d(0,100);

  glColor3d(0,0,1);//z
  glVertex3d(0,0,-100);
  glVertex3d(0,0, 100);
  glEnd();

}

