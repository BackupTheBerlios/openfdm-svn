/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MachNumber_H
#define OpenFDM_MachNumber_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class MachNumber : public Model {
  OPENFDM_OBJECT(MachNumber, Model);
public:
  MachNumber(const std::string& name);
  virtual ~MachNumber(void);

  virtual void output(const Task&, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;

private:
  /// The input port which must provide the fluid velocity vector
  MatrixInputPort mVelocityPort;
  /// The input port which must provide the sensed speed of sound
  RealInputPort mSoundSpeedPort;
  /// The resulting mach number
  RealOutputPort mMachNumberPort;
};

} // namespace OpenFDM

#endif
