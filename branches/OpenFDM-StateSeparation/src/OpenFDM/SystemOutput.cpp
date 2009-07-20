/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "SystemOutput.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_HDF5_H
#include "HDF5SystemOutput.h"
#endif
#include "CSVSystemOutput.h"

namespace OpenFDM {

SystemOutput::~SystemOutput()
{
}

void SystemOutput::setSystem(const System* system)
{
  mSystem = system;
  attachTo(system);
}

SystemOutput*
SystemOutput::newDefaultSystemOutput(const std::string& filename)
{
#ifdef HAVE_HDF5_H
  return new HDF5SystemOutput(filename);
#else
  return new CSVSystemOutput(filename);
#endif
}

} // namespace OpenFDM
