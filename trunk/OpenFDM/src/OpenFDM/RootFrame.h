/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RootFrame_H
#define OpenFDM_RootFrame_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Joint.h"

namespace OpenFDM {

class RootFrame :
    public FreeFrame {
public:
  RootFrame(const std::string& name = std::string());
  virtual ~RootFrame(void);
};

} // namespace OpenFDM

#endif
