/* -*-c++-*- OpenFDM - Copyright (C) 2005-2010 Mathias Froehlich 
 *
 */

#ifndef MainWindow_H
#define MainWindow_H

#include <QtGui/QMainWindow>

class MainWindow : public QMainWindow
{
//   Q_OBJECT
public:
  MainWindow(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~MainWindow(void);

private:
  void setupToolBar(void);
  void setupMenuBar(void);
  void setupDockWindows(void);
  void setupActions(void);

  // All the actions we can have in this application
  QAction* mOpenAction;
  QAction* mSaveAsAction;
  QAction* mImportAction;
  QAction* mQuitAction;

  QAction* mOpenModelBrowserAction;
  QAction* mOpenFrameBrowserAction;
  QAction* mOpen3DViewAction;

  QAction* mAboutAction;
};

#endif
