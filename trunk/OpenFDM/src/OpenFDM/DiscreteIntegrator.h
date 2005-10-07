/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DiscreteIntegrator_H
#define OpenFDM_DiscreteIntegrator_H

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

  virtual void setDiscreteState(const Vector& state, unsigned offset);
  virtual void getDiscreteState(Vector& state, unsigned offset) const;

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
