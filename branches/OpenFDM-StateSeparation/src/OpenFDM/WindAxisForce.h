/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WindAxisForce_H
#define OpenFDM_WindAxisForce_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class WindAxisForce : public Model {
  OPENFDM_OBJECT(WindAxisForce, Model);
public:
  WindAxisForce(const std::string& name);
  virtual ~WindAxisForce(void);

  virtual void output(const Task&, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;

  // FIXME insert the enable/disable stuff for the 3 inputs

private:
  /// The angle of attack input
  RealInputPort mAlphaPort;
  /// The side slip angle input
  RealInputPort mBetaPort;
  /// The drag force input
  RealInputPort mDragPort;
  /// The side force input
  RealInputPort mSidePort;
  /// The lift force input
  RealInputPort mLiftPort;

  /// The resulting force in body axis
  MatrixOutputPort mForcePort;
};

} // namespace OpenFDM

#endif
