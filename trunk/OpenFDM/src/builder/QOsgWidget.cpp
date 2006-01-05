/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <QtGui/QWidget>
#include <QtOpenGL/QtOpenGL>

#include <osg/Node>
#include <osg/Timer>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/MatrixManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/SetSceneViewVisitor>
#include <osgGA/StateSetManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/UFOManipulator>
#include <osgUtil/SceneView>

#include "QOsgWidget.h"

class QtEventAdapter
  : public osgGA::GUIEventAdapter {
public:
  QtEventAdapter(EventType eventType, int xMax, int yMax,
                 int x = -1, int y = -1)
  {
    eventType_ = eventType;
    key_ = -1;
    button_ = -1;
    xMax_ = static_cast<float>(xMax);
    yMax_ = static_cast<float>(yMax);
    x_ = x;
    y_ = y;
    buttonMask_ = 0;
    modKeyMask_ = 0;
    time_ = QOsgWidget::getOsgTime();
  }
  QtEventAdapter(const QMouseEvent* mouseEvent, int xMax, int yMax)
  {
    switch (mouseEvent->type()) {
    case QEvent::MouseButtonPress:
      eventType_ = PUSH;
      break;
    case QEvent::MouseButtonRelease:
      eventType_ = RELEASE;
      break;
    case QEvent::MouseButtonDblClick:
      eventType_ = DOUBLECLICK;
      break;
    case QEvent::MouseMove:
      eventType_ = DRAG;
      break;
    default:
      eventType_ = NONE;
      break;
    }

    key_ = -1;

    switch (mouseEvent->button()) {
    case Qt::LeftButton:
      button_ = LEFT_MOUSE_BUTTON;
      break;
    case Qt::RightButton:
      button_ = RIGHT_MOUSE_BUTTON;
      break;
    case Qt::MidButton:
      button_ = MIDDLE_MOUSE_BUTTON;
      break;
    default:
      button_ = -1;
      break;
    }

    xMax_ = static_cast<float>(xMax);
    yMax_ = static_cast<float>(yMax);

    x_ = mouseEvent->x();
    y_ = mouseEvent->y();

    buttonMask_ = 0;
    if (mouseEvent->buttons() & Qt::LeftButton)
      buttonMask_ |= LEFT_MOUSE_BUTTON;
    if (mouseEvent->buttons() & Qt::RightButton)
      buttonMask_ |= RIGHT_MOUSE_BUTTON;
    if (mouseEvent->buttons() & Qt::MidButton)
      buttonMask_ |= MIDDLE_MOUSE_BUTTON;

    modKeyMask_ = 0;
    if (mouseEvent->modifiers() & Qt::ShiftModifier)
      modKeyMask_ |= MODKEY_SHIFT;
    if (mouseEvent->modifiers() & Qt::ControlModifier)
      modKeyMask_ |= MODKEY_CTRL;
    if (mouseEvent->modifiers() & Qt::AltModifier)
      modKeyMask_ |= MODKEY_ALT;
    if (mouseEvent->modifiers() & Qt::MetaModifier)
      modKeyMask_ |= MODKEY_META;
    if (mouseEvent->modifiers() & Qt::KeypadModifier)
      modKeyMask_ |= MODKEY_NUM_LOCK;

    time_ = QOsgWidget::getOsgTime();
  }
  QtEventAdapter(const QKeyEvent* keyEvent)
  {
    switch (keyEvent->type()) {
    case QEvent::KeyPress:
      eventType_ = KEYDOWN;
      break;
    case QEvent::KeyRelease:
      eventType_ = KEYUP;
      break;
    default:
      eventType_ = NONE;
      break;
    }

    key_ = -1;
    QString txt = keyEvent->text();
    if (txt.length()) {
      char c0 = txt[0].toLatin1();
      if (c0)
        key_ = c0;
    }

    button_ = -1;

    xMax_ = 1.0f;
    yMax_ = 1.0f;

    x_ = -1;
    y_ = -1;

    buttonMask_ = 0;

    modKeyMask_ = 0;
    if (keyEvent->modifiers() & Qt::ShiftModifier)
      modKeyMask_ |= MODKEY_SHIFT;
    if (keyEvent->modifiers() & Qt::ControlModifier)
      modKeyMask_ |= MODKEY_CTRL;
    if (keyEvent->modifiers() & Qt::AltModifier)
      modKeyMask_ |= MODKEY_ALT;
    if (keyEvent->modifiers() & Qt::MetaModifier)
      modKeyMask_ |= MODKEY_META;
    if (keyEvent->modifiers() & Qt::KeypadModifier)
      modKeyMask_ |= MODKEY_NUM_LOCK;

    time_ = QOsgWidget::getOsgTime();
  }

protected:
  virtual ~QtEventAdapter(void) {}

  virtual EventType getEventType(void) const { return eventType_; }
  virtual int getKey(void) const { return key_; }
  virtual int getButton(void) const { return button_; }
  virtual float getXmin(void) const { return 0.0f; }
  virtual float getXmax(void) const { return xMax_; }
  virtual float getYmin(void) const { return 0.0f; }
  virtual float getYmax(void) const { return yMax_; }
  virtual float getX(void) const { return x_; }
  virtual float getY(void) const { return y_; }
  virtual unsigned int getButtonMask(void) const { return buttonMask_; }
  virtual unsigned int getModKeyMask(void) const { return modKeyMask_; }
  virtual double time(void) const { return time_; }
  
private:
  EventType eventType_;
  int key_;
  int button_;
  float xMax_;
  float yMax_;
  float x_;
  float y_;
  unsigned int buttonMask_;
  unsigned int modKeyMask_;
  double time_;
};

QOsgWidget::QOsgWidget(QWidget *parent, const QGLWidget* shareWidget,
                       Qt::WFlags f)
  : QGLWidget(parent, shareWidget, f),
    sceneView_(new osgUtil::SceneView),
    keySwitchManip_(new osgGA::KeySwitchMatrixManipulator),
    frameStamp_(new osg::FrameStamp),
    timerId_(0)
{
//   osgGA::CompositeGUIEventHandler* compositeHandler
//     = new osgGA::CompositeGUIEventHandler();

//   compositeHandler->addChild(keySwitchManip_.get());
//   compositeHandler->addChild(new osgGA::StateSetManipulator);
//   guiEventHandler_ = compositeHandler;

  guiEventHandler_ = keySwitchManip_.get();

//   guiEventHandler_ = new osgGA::AnimationPathManipulator();
//   guiEventHandler_ = new osgGA::DriveManipulator();
//   guiEventHandler_ = new osgGA::FlightManipulator();
//   guiEventHandler_ = new osgGA::NodeTrackerManipulator();
//   guiEventHandler_ = new osgGA::StateSetManipulator();
//   guiEventHandler_ = new osgGA::TerrainManipulator();
//   guiEventHandler_ = new osgGA::TrackballManipulator();
//   guiEventHandler_ = new osgGA::UFOManipulator();

  addCameraManipulator(new osgGA::TrackballManipulator());
  addCameraManipulator(new osgGA::DriveManipulator());


}

void
QOsgWidget::setScene(osg::Node* node)
{
  // Attach this node to the scene view.
  sceneView_->setSceneData(node);
  // Set the scene view on the event handlers
  copySceneView();
}

double
QOsgWidget::getOsgTime(void)
{
  return osg::Timer::instance()->getSecondsPerTick()
    * osg::Timer::instance()->tick();
}

void
QOsgWidget::initializeGL()
{
//   cout << __PRETTY_FUNCTION__ << endl;

  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glColorMaterial(GL_BACK, GL_AMBIENT_AND_DIFFUSE);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
  
  sceneView_->setDefaults();
  sceneView_->setFrameStamp(frameStamp_.get());
  sceneView_->setViewport(0, 0, width(), height());

  float ratio = static_cast<float>(width())/static_cast<float>(height());
  sceneView_->setProjectionMatrixAsPerspective(50.0f, ratio, 1.0f, 10000.0f);

  setFocusPolicy(Qt::StrongFocus);
}

void
QOsgWidget::resizeGL(int w, int h)
{
//   cout << __PRETTY_FUNCTION__ << endl;

  if (!isValid())
    return;

  sceneView_->setViewport(0, 0, width(), height());
  float ratio = static_cast<float>(width())/static_cast<float>(height());
  sceneView_->setProjectionMatrixAsPerspective(50.0f, ratio, 1.0f, 10000.0f);
}

void
QOsgWidget::paintGL()
{
//   cout << __PRETTY_FUNCTION__ << endl;

  if (!isValid())
    return;

  frameStamp_->setFrameNumber(frameStamp_->getFrameNumber()+1);
  frameStamp_->setReferenceTime(getOsgTime());

  // Update the model view on the scene view.
  if (keySwitchManip_.valid())
    sceneView_->setViewMatrix(keySwitchManip_->getInverseMatrix());

  sceneView_->update();
  sceneView_->cull();
  sceneView_->draw();
}

void
QOsgWidget::mousePressEvent(QMouseEvent* mouseEvent)
{
  handleOsgEvent(new QtEventAdapter(mouseEvent, width(), height()));
}

void
QOsgWidget::mouseReleaseEvent(QMouseEvent* mouseEvent)
{
  handleOsgEvent(new QtEventAdapter(mouseEvent, width(), height()));
}

void
QOsgWidget::mouseDoubleClickEvent(QMouseEvent* mouseEvent)
{
  handleOsgEvent(new QtEventAdapter(mouseEvent, width(), height()));
}

void
QOsgWidget::mouseMoveEvent(QMouseEvent* mouseEvent)
{
  handleOsgEvent(new QtEventAdapter(mouseEvent, width(), height()));
}

void
QOsgWidget::keyPressEvent(QKeyEvent* keyEvent)
{
  handleOsgEvent(new QtEventAdapter(keyEvent));
}

void
QOsgWidget::keyReleaseEvent(QKeyEvent* keyEvent)
{
  handleOsgEvent(new QtEventAdapter(keyEvent));
}

void
QOsgWidget::timerEvent(QTimerEvent* timerEvent)
{
  // Only respond to timer events we know of.
  if (timerEvent->timerId() != timerId_)
    return;

  // Get the widget-relative mouse coords
  QPoint p = mapFromGlobal(QCursor::pos());
  handleOsgEvent(new QtEventAdapter(QtEventAdapter::FRAME, width(), height(),
                                    p.x(), p.y()));
}

void
QOsgWidget::requestRedraw(void)
{
  updateGL();
}

void
QOsgWidget::requestContinuousUpdate(bool needed)
{
  if (needed) {
    if (timerId_ == 0)
      timerId_ = startTimer(10);
  } else {
    if (timerId_ != 0) {
      killTimer(timerId_);
      timerId_ = 0;
    }
  }
}

void
QOsgWidget::requestWarpPointer(float x, float y)
{
  QPoint point(static_cast<int>(x), static_cast<int>(y));
  QCursor::setPos(mapToGlobal(point));
}

void
QOsgWidget::copySceneView(void)
{
  // Make sure it is deleted afterwards. Not sure about this ...
  QtEventAdapter* eventAdapter
    = new QtEventAdapter(QtEventAdapter::NONE, width(), height());

  // Sets up view matrices in the GUIEventHandlers from the view present
  // the scene view
  osgGA::SetSceneViewVisitor ssvv(eventAdapter, this, sceneView_.get());
  guiEventHandler_->accept(ssvv);
}

void
QOsgWidget::handleOsgEvent(QtEventAdapter* eventAdapter)
{
  osg::ref_ptr<QtEventAdapter> eventAdapterRef = eventAdapter;
  guiEventHandler_->handle(*eventAdapter, *this, 0, 0);
}

unsigned int
QOsgWidget::addCameraManipulator(osgGA::MatrixManipulator* matrixManip)
{
  if (!matrixManip)
    return 0xfffff;
  
  unsigned int num = keySwitchManip_->getNumMatrixManipulators();
  keySwitchManip_->addNumberedMatrixManipulator(matrixManip);
    
  return num;
}
