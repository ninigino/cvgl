

#include <GL/glut.h>

//---------- 各種外部変数 ---------------//
// 制御点
float CtlPoint[]=
{
	0.0, 0.0, 0.0,  0.2, 0.3, 0.0,  0.4, 0.3, 0.0,  0.6, 0.0, 0.0,
	0.0, 0.2, 0.2,  0.2, 0.5, 0.2,  0.4, 0.5, 0.2,  0.6, 0.2, 0.2,
	0.0, 0.2, 0.4,  0.2, 0.5, 0.4,  0.4, 0.5, 0.4,  0.6, 0.2, 0.4,
	0.0, 0.0, 0.6,  0.2, 0.3, 0.6,  0.4, 0.3, 0.6,  0.6, 0.0, 0.6,
};
//分割数
const float slice = 0.1;

//------------ プロトタイプ宣言 --------------------//
void display();
void reshape(int w, int h);
void timer(int value);

void DRAW_XYZ();
void DrawCurve();

//-----------------OpenGLの初期設定------------------------//
void GLUT_INIT()
{
	glutInitDisplayMode(GLUT_RGBA| GLUT_DOUBLE | GLUT_DEPTH); //ダブルバッファ、Zバッファ
	glutInitWindowSize(300,300);
	glutCreateWindow("Draw Face");
}

void GLUT_CALL_FUNC()
{
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutTimerFunc(0,timer,17);
}

void MY_INIT()
{
	glClearColor(1.0, 1.0, 1.0, 1.0);
	
	//---エバリュエータの設定---//
	glMap2f(GL_MAP2_VERTEX_3, 0.0, 1.0, 3, 4, 0.0, 1.0, 3*4, 4,CtlPoint);
	glEnable(GL_MAP2_VERTEX_3); //有効化
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
}

//---------------- メイン関数 ------------------//
int main(int argc, char **argv)
{
	glutInit(&argc,argv);
	
	GLUT_INIT();
	GLUT_CALL_FUNC();
	MY_INIT();

	glutMainLoop();

	return 0;
}

//--------------- ここから各種コールバック ------------------//
void display()
{

	static int r = 0;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(1, 1.0, 2.0, 0, 0.0, 0.0, 0.0, 1.0, 0.0);

	glEnable(GL_DEPTH_TEST);
	
	glRotatef(static_cast<float>(r),0,1,0);
	DRAW_XYZ();
	DrawCurve();

	glutSwapBuffers();
	
	
	if(++r > 360) r = 0;

}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION); //行列モード切替
	glLoadIdentity();  //行列初期化
	gluPerspective(30.0, (double)w / (double)h, 1.0, 100.0);
	glMatrixMode(GL_MODELVIEW); //行列モード切替
	
}

void timer(int t)
{
	glutPostRedisplay();
	glutTimerFunc(t,timer,17); //タイマー関数
}

//-------------- ここから各種関数 ---------------//
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

void DrawCurve()
{
	glColor3f(1,0,1);
	glBegin(GL_QUADS);
	for(float v = 0; v < 1; v += slice){
		for(float u = 0; u < 1; u += slice){
			glEvalCoord2f(u,v);
			glEvalCoord2f(u+slice,v);
			glEvalCoord2f(u+slice,v+slice);
			glEvalCoord2f(u,v+slice);
		}
	}
	glEnd();

	glColor3f(1,1,1);
}

