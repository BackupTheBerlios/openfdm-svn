/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
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

class MultiBodySystem :
    public Model {
public:
  MultiBodySystem(RootFrame* rootFrame);
  virtual ~MultiBodySystem(void);

  /** Sets the state of this multibody system from the state vector state.
   */
  void setEvalState(const Vector& state);
  /** Sets the state of this multibody system from the state vector state
      and returns the time derivative in deriv.
   */
  void computeStateDeriv(real_type t, const Vector& state, Vector& deriv);
  
  virtual void setState(const Vector& state, unsigned offset);
  virtual void getState(Vector& state, unsigned offset) const;
  virtual void getStateDeriv(Vector& stateDeriv, unsigned offset);

  virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);
  virtual void update(const TaskInfo& taskInfo);
private:
  shared_ptr<RootFrame> mRootFrame;
};

} // namespace OpenFDM

#endif
