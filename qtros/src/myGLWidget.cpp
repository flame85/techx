#include <QtGui/QMouseEvent>
#include "../include/qtros/myGLWidget.hpp"
#include <GL/gl.h>
#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>

MyGLWidget::MyGLWidget(QWidget *parent) : QGLWidget(parent) {
   setMouseTracking(true);
    //glutInit(&argc,argv);
}

void MyGLWidget::initializeGL() {
   glDisable(GL_TEXTURE_2D);
   glDisable(GL_DEPTH_TEST);
   glDisable(GL_COLOR_MATERIAL);
   glEnable(GL_BLEND);
   glEnable(GL_POLYGON_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glClearColor(0, 0, 0, 0);
   printf("initializedGL\n");
}
void MyGLWidget::resizeGL(int w, int h){
   glViewport(0, 0, w, h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluOrtho2D(0, w, 0, h); // set origin to bottom left corner
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   printf("resizeGL\n");
}

void MyGLWidget::paintGL() {
   glClear(GL_COLOR_BUFFER_BIT);
   glColor3f(1,0,0);
   glBegin(GL_POLYGON);
   glVertex2f(0,0);
   glVertex2f(100,500);
   glVertex2f(500,100);
   glEnd();
   printf("paintGL\n");
}

void MyGLWidget::drawAxis(int scale)
{
   printf("draw axies\n");
   int nScale = -1 * scale;
    glBegin(GL_LINES);
    glColor4f (1, 0, 0, 1.0);
    glVertex3f(nScale, 0, 0);
    glColor4f (1, 0, 0, 0.0);
    glVertex3f(scale, 0, 0);
    glColor4f (0, 1, 0, 1.0);
    glVertex3f(0, nScale, 0);
    glColor4f (0, 1, 0, 0.0);
    glVertex3f(0, scale, 0);
    glColor4f (0, 0, 1, 1.0);
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
}
void MyGLWidget::mouseMoveEvent(QMouseEvent *event) {
   printf("%d, %d\n", event->x(), event->y());
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
