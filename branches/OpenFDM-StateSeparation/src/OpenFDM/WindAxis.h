/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_WindAxis_H
#define OpenFDM_WindAxis_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class WindAxis : public Model {
  OPENFDM_OBJECT(WindAxis, Model);
public:
  WindAxis(const std::string& name);
  virtual ~WindAxis(void);

  virtual void output(const Task&, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;

private:
  /// The input port which must provide the fluid velocity 6 vector
  MatrixInputPort mVelocityPort;

  /// The resulting angle of attack output
  RealOutputPort mAlphaPort;
  /// The resulting angle of attack derivative output
  RealOutputPort mAlphaDotPort;
  /// The resulting side slip angle output
  RealOutputPort mBetaPort;
  /// The resulting side slip angle derivative output
  RealOutputPort mBetaDotPort;
  /// The resulting true airspeed output
  RealOutputPort mAirSpeedPort;
};

} // namespace OpenFDM

#endif
