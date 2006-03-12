/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "SharedPtr.h"
#include "SampleTime.h"
#include "Model.h"
#include "TaskInfo.h"

namespace OpenFDM {

void
TaskInfo::output(void) const
{
  ModelList::const_iterator it = mModelList.begin();
  for (; it != mModelList.end(); ++it)
    (*it)->outputIfEnabled(*this);
}

void
TaskInfo::update(void) const
{
  ModelList::const_iterator it = mModelList.begin();
  for (; it != mModelList.end(); ++it)
    (*it)->updateIfEnabled(*this);
}

} // namespace OpenFDM
