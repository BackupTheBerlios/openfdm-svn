/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixStateInfo_H
#define OpenFDM_MatrixStateInfo_H

#include "Matrix.h"
#include "TemplateContinousStateInfo.h"

namespace OpenFDM {

class MatrixStateInfo : public TemplateContinousStateInfo<Matrix> {
};

} // namespace OpenFDM

#endif
