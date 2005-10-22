/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TimeDerivative_H
#define OpenFDM_TimeDerivative_H

#include "Assert.h"
#include "Types.h"
#include "Object.h"
#include "Model.h"
#include "Expression.h"

namespace OpenFDM {

class TimeDerivative :
    public Model {
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
};

} // namespace OpenFDM

#endif
