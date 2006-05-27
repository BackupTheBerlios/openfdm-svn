/* -*-c++-*- OpenFDM - Copyright (C) 2005-2006 Mathias Froehlich 
 *
 */

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

#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/XMLDumpModelVisitor.h>
#include <JSBSim/JSBSimReader.h>

using OpenFDM::ReaderWriter;
using OpenFDM::JSBSimReader;

#include <iostream>

#include "Geometries.h"
#include "QOsgWidget.h"

#include "FrameItem.h"
#include "ModelItem.h"
#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags)
{
  setObjectName("MainWindow");
  setWindowTitle("OpenFDM Flightmodel Builder");
  
  // Make sure the actions are present
  setupActions();
  
  setupToolBar();
  setupMenuBar();
  setupDockWindows();
  
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

MainWindow::~MainWindow(void)
{
}

void
MainWindow::setupToolBar()
{
//     toolbar = new ToolBar(this);
//     toolbar->setAllowedAreas(Qt::ToolBarAreaTop | Qt::ToolBarAreaBottom);
//     addToolBar(toolbar);
}

void
MainWindow::setupMenuBar()
{
  QMenu *menu = menuBar()->addMenu(tr("&File"));
  menu->addAction(mOpenAction);
  menu->addAction(mSaveAsAction);
  menu->addSeparator();
  menu->addAction(mImportAction);
  menu->addSeparator();
  menu->addAction(mQuitAction);
  
  menu = menuBar()->addMenu(tr("&Tools"));
  menu->addAction(mOpenModelBrowserAction);
  menu->addAction(mOpenFrameBrowserAction);
  menu->addAction(mOpen3DViewAction);
  
  menu = menuBar()->addMenu(tr("&Help"));
  menu->addAction(mAboutAction);
}

void
MainWindow::setupDockWindows()
{
  // Try to read JSBSim legacy files.
  JSBSimReader reader;
  
  reader.addAircraftPath("/home/flightgear/sw/share/FlightGear/Aircraft/FA-18/");
  reader.addEnginePath("/home/flightgear/sw/share/FlightGear/Aircraft/FA-18/Engines/");
  
  reader.loadAircraft("FA-18-cross.xml");
  if (reader.getErrorState()) {
    const ReaderWriter::StringList errors = reader.getErrors();
    ReaderWriter::StringList::const_iterator it;
    for (it = errors.begin(); it != errors.end(); ++it)
      std::cerr << *it << std::endl;
    return;
  }
  
  // Ok, now the Vehicle here contains the imported data
  // When the reflection stuff is ready, we can dump that data to a
  // native format ...
  reader.getVehicle()->getSystem()->init();
  
  ModelItem* model = new ModelItem;
  model->setSystem(reader.getVehicle()->getSystem());

  QDockWidget* dockWidget = new QDockWidget(this);
  QTreeView* treeView = new QTreeView(dockWidget);
  treeView->setModel(model);
  dockWidget->setWidget(treeView);
  addDockWidget(Qt::LeftDockWidgetArea, dockWidget);


  FrameItem* frameModel = new FrameItem;
  frameModel->setRootFrame(reader.getVehicle()->getSystem()->getEnvironment()->getRootFrame());

  dockWidget = new QDockWidget(this);
  treeView = new QTreeView(dockWidget);
  treeView->setModel(frameModel);
  dockWidget->setWidget(treeView);
  addDockWidget(Qt::RightDockWidgetArea, dockWidget);
}

void
MainWindow::setupActions(void)
{
  mOpenAction = new QAction(tr("&Open..."), this);
  mOpenAction->setShortcut(tr("Ctrl+O"));
  //     connect(mOpenAction, SIGNAL(triggered()), this, SLOT(open()));
  
  mSaveAsAction = new QAction(tr("&Save As..."), this);
  mSaveAsAction->setShortcut(tr("Ctrl+S"));
  //     connect(mSaveAsAction, SIGNAL(triggered()), this, SLOT(saveAs()));
  
  mImportAction = new QAction(tr("&Import..."), this);
  mImportAction->setShortcut(tr("Ctrl+I"));
  //     connect(mImportAction, SIGNAL(triggered()), this, SLOT(saveAs()));
  
  mQuitAction = new QAction(tr("&Quit"), this);
  mQuitAction->setShortcut(tr("Ctrl+Q"));
  connect(mQuitAction, SIGNAL(triggered()), this, SLOT(close()));
  
  mOpenModelBrowserAction = new QAction(tr("Open &Model Browser"), this);
  mOpenModelBrowserAction->setShortcut(tr("Ctrl+M"));
  //     connect(mOpenModelBrowserAction, SIGNAL(triggered()), this, SLOT(close()));
  
  mOpenFrameBrowserAction = new QAction(tr("Open &Frame Browser"), this);
  mOpenFrameBrowserAction->setShortcut(tr("Ctrl+F"));
  //     connect(mOpenFrameBrowserAction, SIGNAL(triggered()), this, SLOT(close()));
  
  mOpen3DViewAction = new QAction(tr("Open &3D View"), this);
  mOpen3DViewAction->setShortcut(tr("Ctrl+3"));
  //     connect(mOpen3DViewAction, SIGNAL(triggered()), this, SLOT(close()));
  
  mAboutAction = new QAction(tr("&About"), this);
  connect(mAboutAction, SIGNAL(triggered()), this, SLOT(about()));
}
