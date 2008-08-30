/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Gain_H
#define OpenFDM_Gain_H

#include <string>

#include "Model.h"

namespace OpenFDM {

class Gain : public Model {
  OPENFDM_OBJECT(Gain, Model);
public:
  Gain(const std::string& name);
  virtual ~Gain(void);

  virtual bool alloc(LeafContext& leafContext) const;
  virtual void output(const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;
  virtual bool dependsOn(const PortId& in, const PortId& out) const;

  const real_type& getGain(void) const;
  void setGain(const real_type& gain);

private:
  MatrixInputPort mInputPort;
  MatrixOutputPort mOutputPort;
  real_type mGain;
};

} // namespace OpenFDM

#endif
