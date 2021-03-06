/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DiscreteIntegrator_H
#define OpenFDM_DiscreteIntegrator_H

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class DiscreteIntegrator : public Model {
  OPENFDM_OBJECT(DiscreteIntegrator, Model);
public:
  DiscreteIntegrator(const std::string& name);
  virtual ~DiscreteIntegrator(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);
  virtual void update(const TaskInfo& taskInfo);

  virtual void setDiscreteState(const StateStream& state);
  virtual void getDiscreteState(StateStream& state) const;

  virtual bool dependsDirectOn(Model* model);

  const Matrix& getInitialValue(void) const;
  void setInitialValue(const Matrix& value);

  const Matrix& getMinSaturation(void) const;
  void setMinSaturation(const Matrix& value);

  const Matrix& getMaxSaturation(void) const;
  void setMaxSaturation(const Matrix& value);

  const Matrix& getIntegralOutput(void) const;

private:
  /// Holds the current output.
  Matrix mIntegralOutput;
  /// Holds the current integral state.
  Matrix mIntegralState;
  /// Holds the current integral initial state.
  Matrix mInitialValue;
  /// Holds the minimum saturation
  Matrix mMinSaturation;
  /// Holds the maximum saturation
  Matrix mMaxSaturation;
  /// Holds a matrix handle to the integrators input
  MatrixPortHandle mDerivativePort;
  /// Holds a matrix handle to the integrators initial value input
  MatrixPortHandle mInitialValuePort;
};

} // namespace OpenFDM

#endif
