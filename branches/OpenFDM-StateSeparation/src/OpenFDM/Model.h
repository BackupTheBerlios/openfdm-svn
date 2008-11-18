/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Model_H
#define OpenFDM_Model_H

#include <string>

#include "OpenFDMConfig.h"
#include "Assert.h"
#include "AbstractModel.h"
#include "MatrixInputPort.h"
#include "MatrixOutputPort.h"
#include "RealInputPort.h"
#include "RealOutputPort.h"

namespace OpenFDM {

class ContinousTask;
class DiscreteTask;
class DiscreteStateValueVector;
class ContinousStateValueVector;
class PortValueList;
class ContinousStateValueVector;
class Task;
class ModelContext;

class Model : public AbstractModel {
  OPENFDM_OBJECT(Model, AbstractModel);
public:
  Model(const std::string& name);
  virtual ~Model();

  virtual ModelContext* newModelContext(PortValueList& portValueList) const;

  // FIXME: May be we want to collapse all state values in one
  // argument? May be it is sufficient to have a const and non const version??
  // FIXME???
  // const TaskInfo& taskInfo
  virtual bool alloc(LeafContext&) const // = 0;
  { return true; }

  virtual void init(const Task&,DiscreteStateValueVector&,
                    ContinousStateValueVector&, const PortValueList&) const
  { }
  virtual void update(const DiscreteTask&, DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      const PortValueList&) const { }
  virtual void output(const Task&,const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList&) const { }
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList&,
                          ContinousStateValueVector&) const { }

protected:
  MatrixOutputPort
  newMatrixOutputPort(const std::string& name, const Size& size = Size(0, 0))
  { return MatrixOutputPort(this, name, size); }
  MatrixInputPort
  newMatrixInputPort(const std::string& name, bool directInput,
                     const Size& size = Size(0, 0))
  { return MatrixInputPort(this, name, size, directInput); }
  RealOutputPort
  newRealOutputPort(const std::string& name)
  { return RealOutputPort(this, name); }
  RealInputPort
  newRealInputPort(const std::string& name, bool directInput)
  { return RealInputPort(this, name, directInput); }

private:
  class Context;
};

} // namespace OpenFDM

#endif
