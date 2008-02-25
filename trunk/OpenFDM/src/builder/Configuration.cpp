/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Configuration.h"

namespace OpenFDM {

// SharedPtr<Configuration> Configuration::mConfiguration;
Configuration* Configuration::mConfiguration = 0;

Configuration::Configuration(void)
{
}

Configuration::~Configuration(void)
{
}

Configuration*
Configuration::Instance(void)
{
  if (mConfiguration == 0)
    mConfiguration = new Configuration;
  return mConfiguration;
}

QString
Configuration::getSharePath(void) const
{
  // FIXME
  return QString::fromAscii("/home/flightgear/sw/share/OpenFDM/");
}

QString
Configuration::getObjectLibraryPath(void) const
{
  return getSharePath() + QString::fromAscii("objects/");
}

QByteArray
Configuration::getLibraryObject(const char* object) const
{
  QString fileName = getObjectLibraryPath() + QString(object);
  return QFile::encodeName(fileName);
}

} // namespace OpenFDM
