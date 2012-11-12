#ifndef _MyGLWIDGET_H
#define _MyGLWIDGET_H

#include <QtOpenGL/QGLWidget>

class MyGLWidget : public QGLWidget {

    Q_OBJECT // must include this if you use Qt signals/slots

public:
    MyGLWidget(QWidget *parent = NULL);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void drawAxis(int scale);
};

#endif  /* _GLWIDGET_H */
