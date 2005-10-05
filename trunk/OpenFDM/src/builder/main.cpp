#include <QtGui/QApplication>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>

#include <osg/Node>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/CullFace>
#include <osg/Transform>
#include <osg/Quat>
#include <osg/PositionAttitudeTransform>
#include <osgDB/Archive>
#include <osgDB/ReadFile>

#include <iostream>

#include "Geometries.h"
#include "QOsgWidget.h"

class MainWindow : public QMainWindow
{
//   Q_OBJECT
  
  QMenu *dockWindowMenu;
  
public:
  MainWindow(QWidget *parent = 0, Qt::WFlags flags = 0)
    : QMainWindow(parent, flags)
  {
    setObjectName("MainWindow");
    setWindowTitle("OpenFDM Flightmodel Builder");
    
//     setupToolBar();
    setupMenuBar();
//     setupDockWindows();

    if (QGLFormat::hasOpenGL()) {
      QOsgWidget *center = new QOsgWidget(this);
      setCentralWidget(center);

//       osgDB::ReaderWriter::Options* opts = new osgDB::ReaderWriter::Options;
//       opts->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_ALL);
//       osgDB::Registry::instance()->setOptions(opts);
//       center->setScene(osgDB::readNodeFile("/home/frohlich/3D-Models/FA-18/Body/fa-18ab.ac"));

      center->setScene(OpenFDM::genCoordinateBullet());
    }


//     statusBar()->message(tr("Status Bar"));
  }

  
public slots:
  void actionTriggered(QAction *action)
  {
    qDebug("action '%s' triggered", action->text().toLocal8Bit().data());
  }

  
private:
//   void setupToolBar()
//   {
//     toolbar = new ToolBar(this);
//     toolbar->setAllowedAreas(Qt::ToolBarAreaTop | Qt::ToolBarAreaBottom);
//     addToolBar(toolbar);
//   }

  void setupMenuBar()
  {
    QMenu *menu = menuBar()->addMenu(tr("&File"));
    menu->addAction(tr("&Quit"), this, SLOT(close()));
    
//     menuBar()->addMenu(toolbar->menu);
//     dockWindowMenu = menuBar()->addMenu(tr("&Dock windows"));
  }
//   void setupDockWindows();
};

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  MainWindow mwd;
  app.setActiveWindow(&mwd);
  mwd.show();
  return app.exec();
}
