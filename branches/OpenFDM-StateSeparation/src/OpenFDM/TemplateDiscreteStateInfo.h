/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TemplateDiscreteStateInfo_H
#define OpenFDM_TemplateDiscreteStateInfo_H

#include "StateInfo.h"
#include "StateValue.h"
#include "TemplateStateInfo.h"

namespace OpenFDM {

template<typename T>
class TemplateDiscreteStateInfo :
    public TemplateStateInfo<T, StateInfo, StateValue> {
};

} // namespace OpenFDM

#endif
