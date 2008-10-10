/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SimulationTime_H
#define OpenFDM_SimulationTime_H

#include "Model.h"
#include "RealOutputPort.h"

namespace OpenFDM {

class SimulationTime : public Model {
  OPENFDM_OBJECT(SimulationTime, Model);
public:
  SimulationTime(const std::string& name);
  virtual ~SimulationTime(void);

  virtual void output(const Task& task, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;
private:
  RealOutputPort mOutputPort;
};

} // namespace OpenFDM

#endif
