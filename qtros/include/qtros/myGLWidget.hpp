#ifndef _MyGLWIDGET_H
#define _MyGLWIDGET_H

#include <QtOpenGL/QGLWidget>
#include <QMatrix4x4>

class MyGLWidget : public QGLWidget {

    Q_OBJECT // must include this if you use Qt signals/slots

public:
    MyGLWidget(QWidget *parent = NULL);

public slots:
    void resetRobotPose();
    void addTransform(QMatrix4x4 transform);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void drawAxis(int scale = 1);
    void drawPose(float scale = 0.5);
    void drawCoil();
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void drawTraject();
private:
    int xRot, yRot, zRot;
    float xTra, yTra, zTra;
    QPoint lastPos;
    QList<QMatrix4x4> pose_matrices;
};

#endif  /* _GLWIDGET_H */
