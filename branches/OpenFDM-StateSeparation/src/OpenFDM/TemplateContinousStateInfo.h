/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TemplateContinousStateInfo_H
#define OpenFDM_TemplateContinousStateInfo_H

#include "ContinousStateInfo.h"
#include "MatrixStateValue.h"
#include "TemplateStateInfo.h"

namespace OpenFDM {

template<typename T>
class TemplateContinousStateInfo : 
    public TemplateStateInfo<T, ContinousStateInfo, MatrixStateValue> {
};

} // namespace OpenFDM

#endif
