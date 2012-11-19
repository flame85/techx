#include <QtGui/QMouseEvent>
#include "../include/qtros/myGLWidget.hpp"
#include <GL/gl.h>
#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>

const double pi= 3.14159265358979323846;
static void qNormalizeAngle(int &angle) {
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

MyGLWidget::MyGLWidget(QWidget *parent) 
: QGLWidget(parent),
  xRot(0),
  yRot(0),
  zRot(0),
  xTra(0),
  yTra(0),
  zTra(-150)
{
   setMouseTracking(true);
}

void MyGLWidget::initializeGL() {
    //glClearColor(0,0,0,0); 
    glEnable (GL_BLEND); 
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    //glShadeModel(GL_SMOOTH);
    //glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHT0);
    //glEnable(GL_MULTISAMPLE);
    //gluPerspective(99.0/180.0*pi, 1.00, 0.01, 1e9); //1.38 = tan(57/2°)/tan(43/2°)
    ////gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
    //static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    //glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

/*void MyGLWidget::initializeGL() {
   glDisable(GL_TEXTURE_2D);
   glDisable(GL_DEPTH_TEST);
   glDisable(GL_COLOR_MATERIAL);
   glEnable(GL_BLEND);
   glEnable(GL_POLYGON_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glClearColor(0, 0, 0, 0);
   printf("initializedGL\n");
}*/
void MyGLWidget::resizeGL(int w, int h){
   glViewport(0, 0, w, h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
//#ifdef QT_OPENGL_ES_1
//    glOrthof(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
//#else
//    glOrtho(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
//#endif
    //gluPerspective(57.0/180.0*pi, 1.38, 0.01, 1e9); //1.38 = tan(57/2°)/tan(43/2°) as kinect has viewing angles 57 and 43
    float ratio = (float)w / (float) h;
    gluPerspective(pi/4.0, ratio, 0.1, 1e4); 
    glMatrixMode(GL_MODELVIEW);
}

void MyGLWidget::paintGL() {
  if(!this->isVisible()) 
    return;
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glLoadIdentity();
    //Camera transformation
    glTranslatef(xTra, yTra, zTra);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
  /* glColor3f(1,0,0);
   glBegin(GL_POLYGON);
   glVertex2f(0,0);
   glVertex2f(100,500);
   glVertex2f(500,100);
   glEnd();*/
    drawPose(0.15);
   drawAxis(200);
   drawTraject();
   drawCar();
   //drawCoil();
   //drawPose(100);
   printf("traject size %d\n", pose_matrices.size());
}

void MyGLWidget::drawTraject()
{
  for(int i = 0; i<pose_matrices.size(); i++){
     glPushMatrix();
     glMultMatrixd(static_cast<GLdouble*>( (pose_matrices)[i].data() ));
       //works as long as qreal and GLdouble are typedefs to double (might depend on hardware)
     drawPose(0.15);
     glPopMatrix();
  }
}

void MyGLWidget::drawCar()
{ // car
    setMaterial(1.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,50);
    glPushMatrix();
    if(!pose_matrices.isEmpty())
      glMultMatrixd(static_cast<GLdouble*>( (pose_matrices.last()).data() ));
    //draw body
    glPushMatrix();                   // body
    glScalef(0.7,.5, 0.3);
    glColor4f(0.7,0,0,1);       //red car body
    glutSolidCube(1);
    glPopMatrix();
    //draw body end
    
    //draw head
    glPushMatrix();
    glTranslatef(0.2,0,.2);
    glScalef(0.2,.2, 0.2);
    glColor4f(0.,0,0.7,0.5);
    glutSolidCube(1);
    glPopMatrix();
    //draw head end
    
    // draw tyres
    glTranslatef(0,0,-.3);
    glPushMatrix();
    glTranslatef(-.2,-.2,0);
    glRotatef(90, 1.0, 0.0, 0.0);
    glColor4f(0.8,0.8,0.8,0.5);  // gray white tyres
    glutSolidTorus(0.08,.11,8,8);       // wheel
    glTranslatef(.4,0,0);
    glutSolidTorus(0.08,.11,8,8);       // wheel
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-.2,.2,0);
    glRotatef(90, 1.0, 0.0, 0.0);
    glutSolidTorus(0.08,0.11,8,8);       // wheel
    glTranslatef(.4,0,0);
    glutSolidTorus(0.08,0.11,8,8);       // wheel
    glPopMatrix();
    // draw tyres end
     
    glPopMatrix();
}

// Shape For Debuging 
void MyGLWidget::drawCoil() {
    const float nbSteps = 200.0;
    glBegin(GL_QUAD_STRIP);
    for (int i=0; i<nbSteps; ++i) {
        const float ratio = i/nbSteps;
        const float angle = 21.0*ratio;
        const float c = cos(angle);
        const float s = sin(angle);
        const float r1 = 1.0 - 0.8f*ratio;
        const float r2 = 0.8f - 0.8f*ratio;
        const float alt = ratio - 0.5f;
        const float nor = 0.5f;
        const float up = sqrt(1.0-nor*nor);
        glColor3f(1.0-ratio, 0.2f , ratio);
        glNormal3f(nor*c, up, nor*s);
        glVertex3f(r1*c, alt, r1*s+2);
        glVertex3f(r2*c, alt+0.05f, r2*s+2); }
    glEnd();
}

void MyGLWidget::drawPose(float scale){
    glBegin(GL_LINES);
    glColor4f (1, 0, 0, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (1, 0, 0, 0.0);
    glVertex3f(scale, 0, 0);
    glColor4f (0, 1, 0, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0, 1, 0, 0.0);
    glVertex3f(0, scale, 0);
    glColor4f (0, 0, 1, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0, 0, 1, 0.0);
    glVertex3f(0, 0, scale);
    glEnd();
}
void MyGLWidget::drawAxis(int scale)
{
   int nScale = -1 * scale;
    glBegin(GL_LINES);
    glColor4f (1, 0, 0, 1.0); //R for x
    glVertex3f(nScale, 0, 0);
    glColor4f (1, 0, 0, 0.0);
    glVertex3f(scale, 0, 0);
    glColor4f (0, 1, 0, 1.0); //g for y
    glVertex3f(0, nScale, 0);
    glColor4f (0, 1, 0, 0.0);
    glVertex3f(0, scale, 0);
    glColor4f (0, 0, 1, 1.0); //b for z
    glVertex3f(0, 0, nScale);
    glColor4f (0, 0, 1, 0.0);
    glVertex3f(0, 0, scale);
    glEnd();
  glColor3f(1, 1, 1);
  //draw ticks X
  //int step = int((i>(int)m_scale) ? i:(int)m_scale);
  for(float i = (float)nScale; i <= (float)scale; i+=0.5)
  {
     glRasterPos3d(i, 0, 0);
     char index[128];
     sprintf(index, "%g", i);
     for(char* c = index; *c != '\0'; c++)
     {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
     }
  }
  //draw ticks Y
  for(float i = (float)nScale; i <= (float)scale; i+=0.5)
  {
     glRasterPos3d(0, i, 0);
     char index[128];
     sprintf(index, "%g", i);
     for(char* c = index; *c != '\0'; c++)
     {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
     }
  }
  //draw ticks Z
  for(float i = 0; i <= 2; i+=0.5)
  {
     glRasterPos3f(0, 0, i);
     char index[128];
     sprintf(index, "%g", i);
     for(char* c = index; *c != '\0'; c++)
     {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
     }
  }
}

void MyGLWidget::mousePressEvent(QMouseEvent *event) {
    lastPos = event->pos();
   printf("%d, %d\n", event->x(), event->y());
}

void MyGLWidget::wheelEvent(QWheelEvent *event) {
    zTra += ((float)event->delta())/10.0; 
    updateGL();
}

/*void MyGLWidget::mouseMoveEvent(QMouseEvent *event) {
   //printf("%d, %d\n", event->x(), event->y());
}*/
void MyGLWidget::mouseMoveEvent(QMouseEvent *event) {//TODO: consolidate setRotation methods
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot - 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot - 8 * dy);
        setZRotation(zRot + 8 * dx);
    } else if (event->buttons() & Qt::MidButton) {
        xTra += dx/200.0;
        yTra -= dy/200.0;
        updateGL();
    }
    lastPos = event->pos();
}

void MyGLWidget::keyPressEvent(QKeyEvent* event) {
   switch(event->key()) {
      case Qt::Key_Escape:
         close();
         break;
      default:
         event->ignore();
         break;
   }
}

void MyGLWidget::setXRotation(int angle) { 
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        updateGL();
    }
}


void MyGLWidget::setYRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        updateGL();
    }
}

void MyGLWidget::setZRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        updateGL();
    }
}

void MyGLWidget::resetRobotPose() {
  printf("reset button pressed\n");
  xRot = 0;
  yRot = 0;
  zRot = 0;
  xTra = 0;
  yTra = 0;
  zTra = -500;
  pose_matrices.clear();
  updateGL();
}

void MyGLWidget::addTransform(QMatrix4x4 transform){
  //printf("add transform received by glwidget\n");
   pose_matrices.push_back(transform); //keep for later
   /*for(int i = 0; i < 4; ++i)
   {
     for(int j = 0; j <4; ++j)
     {
       printf("%lf ", transform(i,j));
     }
     printf("\n");
   }*/
   updateGL();
}

void MyGLWidget::setMaterial ( GLfloat ambientR, GLfloat ambientG, GLfloat ambientB, 
		   GLfloat diffuseR, GLfloat diffuseG, GLfloat diffuseB, 
		   GLfloat specularR, GLfloat specularG, GLfloat specularB,
		   GLfloat shininess ) {

    GLfloat ambient[] = { ambientR, ambientG, ambientB };
    GLfloat diffuse[] = { diffuseR, diffuseG, diffuseB };
    GLfloat specular[] = { specularR, specularG, specularB };

    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ambient);
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diffuse);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular);
    glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,shininess);
}
