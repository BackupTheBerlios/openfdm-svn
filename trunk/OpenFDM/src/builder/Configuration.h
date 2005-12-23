/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Configuration_H
#define OpenFDM_Configuration_H

#include <QtCore/QString>
#include <QtCore/QByteArray>
#include <QtCore/QFile>

namespace OpenFDM {

class Configuration {
public:
  ~Configuration(void);

  static Configuration* Instance(void);

  QString getSharePath(void) const;
  QString getObjectLibraryPath(void) const;
  QByteArray getLibraryObject(const char* object) const;

private:
  // Can only be created once.
  Configuration(void);

  // They cannot be used.
  Configuration(const Configuration&);
  Configuration& operator= (const Configuration&);

  // The only single instance.
//   static SharedPtr<Configuration> mConfiguration;
  static Configuration* mConfiguration;
};

} // namespace OpenFDM

#endif
