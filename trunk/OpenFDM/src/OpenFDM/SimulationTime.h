/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SimulationTime_H
#define OpenFDM_SimulationTime_H

#include "Model.h"

namespace OpenFDM {

class SimulationTime : public Model {
  OPENFDM_OBJECT(SimulationTime, Model);
public:
  SimulationTime(const std::string& name);
  virtual ~SimulationTime(void);

  /// Double dispatch helper for the system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getOutputValue(void) const;

private:
  real_type mOutputValue;
};

} // namespace OpenFDM

#endif
