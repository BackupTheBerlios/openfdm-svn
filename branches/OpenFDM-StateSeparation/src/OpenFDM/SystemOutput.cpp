/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "SystemOutput.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_HDF5_H
#include "HDF5SystemOutput.h"
#endif

namespace OpenFDM {

SystemOutput::~SystemOutput()
{
}

SystemOutput*
SystemOutput::newDefaultSystemOutput(const std::string& filename)
{
#ifdef HAVE_HDF5_H
  return new HDF5SystemOutput(filename);
#else
  return 0;
#endif
}

} // namespace OpenFDM
