/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Integrator_H
#define OpenFDM_Integrator_H

#include "Assert.h"
#include "Types.h"
#include "Object.h"
#include "Model.h"

namespace OpenFDM {

class DiscreteIntegrator :
    public Model {
public:
  DiscreteIntegrator(const std::string& name);
  virtual ~DiscreteIntegrator(void);

  virtual bool init(void);
  virtual void output(void);
  virtual void update(real_type dt);

  const Matrix& getInitialValue(void) const;
  void setInitialValue(const Matrix& value);

  const Matrix& getIntegralOutput(void) const;

private:
  /// Holds the current output.
  Matrix mIntegralOutput;
  /// Holds the current integral state.
  Matrix mIntegralState;
  /// Holds the current integral initial state.
  Matrix mInitialValue;
};

} // namespace OpenFDM

#endif
