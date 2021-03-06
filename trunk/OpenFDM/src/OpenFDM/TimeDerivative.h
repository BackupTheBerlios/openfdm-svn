/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TimeDerivative_H
#define OpenFDM_TimeDerivative_H

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class TimeDerivative : public Model {
  OPENFDM_OBJECT(TimeDerivative, Model);
public:
  TimeDerivative(const std::string& name);
  virtual ~TimeDerivative(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);
  virtual void update(const TaskInfo& taskInfo);

  const Matrix& getDerivativeOutput(void) const;

private:
  /// Holds the current output.
  Matrix mDerivativeOutput;
  Matrix mPastInput;
  double mDt;

  /// The input port handle
  MatrixPortHandle mInputPort;
};

} // namespace OpenFDM

#endif
