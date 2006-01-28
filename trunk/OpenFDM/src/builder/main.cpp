#include <QtGui/QApplication>
#include "MainWindow.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  MainWindow mainWindow;
  app.setActiveWindow(&mainWindow);
  mainWindow.show();
  return app.exec();
}
