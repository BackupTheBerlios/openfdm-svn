/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DiscreteStateValueVector_H
#define OpenFDM_DiscreteStateValueVector_H

#include "TemplateValueVector.h"
#include "StateInfo.h"
#include "StateValue.h"

namespace OpenFDM {

class DiscreteStateValueVector :
    public TemplateValueVector<StateInfo, StateValue> {
public:
};

} // namespace OpenFDM

#endif
