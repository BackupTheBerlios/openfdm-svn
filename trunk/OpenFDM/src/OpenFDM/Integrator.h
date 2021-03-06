/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Integrator_H
#define OpenFDM_Integrator_H

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class Integrator : public Model {
  OPENFDM_OBJECT(Integrator, Model);
public:
  Integrator(const std::string& name);
  virtual ~Integrator(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  virtual void setState(const StateStream& state);
  virtual void getState(StateStream& state) const;
  virtual void getStateDeriv(StateStream& stateDeriv);

  const Matrix& getInitialValue(void) const;
  void setInitialValue(const Matrix& value);

  const Matrix& getIntegralOutput(void) const;

private:
  /// The handle to the input port
  MatrixPortHandle mDerivativeHandle;
  /// Holds the current output.
  Matrix mIntegralOutput;
  /// Holds the current integral state.
  Matrix mIntegralState;
  /// Holds the current integral initial state.
  Matrix mInitialValue;
};

} // namespace OpenFDM

#endif
