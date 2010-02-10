/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixStateInfo_H
#define OpenFDM_MatrixStateInfo_H

#include "Matrix.h"
#include "TemplateContinousStateInfo.h"

namespace OpenFDM {

class MatrixStateInfo : public TemplateContinousStateInfo<Matrix> {
};

class Vector1StateInfo : public TemplateContinousStateInfo<Vector1> {
};

class Vector2StateInfo : public TemplateContinousStateInfo<Vector2> {
};

class Vector3StateInfo : public TemplateContinousStateInfo<Vector3> {
};

class Vector4StateInfo : public TemplateContinousStateInfo<Vector4> {
};

class Vector6StateInfo : public TemplateContinousStateInfo<Vector6> {
};

} // namespace OpenFDM

#endif
