/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TimeDerivative_H
#define OpenFDM_TimeDerivative_H

#include "Types.h"
#include "Model.h"
#include "TemplateDiscreteStateInfo.h"

namespace OpenFDM {

class TimeDerivative : public Model {
  OPENFDM_OBJECT(TimeDerivative, Model);
public:
  TimeDerivative(const std::string& name);
  virtual ~TimeDerivative(void);

  virtual bool alloc(ModelContext&) const;
  virtual void init(const Task&,DiscreteStateValueVector& discreteState,
                    ContinousStateValueVector&, const PortValueList&) const;
  virtual void output(const Task&,const DiscreteStateValueVector& discreteState,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;
  virtual void update(const DiscreteTask&, DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      const PortValueList&) const;

private:
  typedef TemplateDiscreteStateInfo<Matrix> MatrixStateInfo;

  MatrixInputPort mInputPort;
  MatrixOutputPort mOutputPort;
  SharedPtr<MatrixStateInfo> mDerivativeStateInfo;
  SharedPtr<MatrixStateInfo> mOldValueStateInfo;
  SharedPtr<MatrixStateInfo> mOldTStateInfo;
};

} // namespace OpenFDM

#endif
