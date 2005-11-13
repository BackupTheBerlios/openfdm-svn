/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Integrator_H
#define OpenFDM_Integrator_H

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class Integrator :
    public Model {
public:
  Integrator(const std::string& name);
  virtual ~Integrator(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  virtual void setState(const Vector& state, unsigned offset);
  virtual void getState(Vector& state, unsigned offset) const;
  virtual void getStateDeriv(Vector& stateDeriv, unsigned offset);

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
