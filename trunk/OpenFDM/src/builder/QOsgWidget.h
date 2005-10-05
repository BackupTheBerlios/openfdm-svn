/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_QOsgWidget_H
#define OpenFDM_QOsgWidget_H

#include <QtGui/QWidget>
#include <QtOpenGL/QtOpenGL>

#include <osg/ref_ptr>
#include <osg/Node>
#include <osg/FrameStamp>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIActionAdapter>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/MatrixManipulator>
#include <osgUtil/SceneView>

class QtEventAdapter;

class QOsgWidget
  : public QGLWidget, protected osgGA::GUIActionAdapter {
public:
  QOsgWidget(QWidget *parent = 0, const QGLWidget* shareWidget = 0,
             Qt::WFlags f = 0);

  void setScene(osg::Node* node);

  // FIXME: andere API ...
  unsigned int addCameraManipulator(osgGA::MatrixManipulator* matrixManip);

  static double getOsgTime(void);

protected:
  // Event handlers for qt.
  virtual void initializeGL(void);
  virtual void resizeGL(int w, int h);
  virtual void paintGL(void);

  virtual void mousePressEvent(QMouseEvent* mouseEvent);
  virtual void mouseReleaseEvent(QMouseEvent* mouseEvent);
  virtual void mouseDoubleClickEvent(QMouseEvent* mouseEvent);
  virtual void mouseMoveEvent(QMouseEvent* mouseEvent);
  virtual void keyPressEvent(QKeyEvent* keyEvent);
  virtual void keyReleaseEvent(QKeyEvent* keyEvent);
  virtual void timerEvent(QTimerEvent *);
  
  // Callbacks from the osg event handling code.
  virtual void requestRedraw(void);
  virtual void requestContinuousUpdate(bool needed);
  virtual void requestWarpPointer(float x, float y);

protected:
  void copySceneView(void);
  void handleOsgEvent(QtEventAdapter* eventAdapter);

private:
  osg::ref_ptr<osgUtil::SceneView> sceneView_;

  osg::ref_ptr<osgGA::GUIEventHandler> guiEventHandler_;

//   osg::ref_ptr<osgGA::MatrixManipulator>          matrixManipulator_;

  osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keySwitchManip_;

  osg::ref_ptr<osg::FrameStamp> frameStamp_;

  int timerId_;
};

#endif
