/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Model_H
#define OpenFDM_Model_H

#include <string>

#include "OpenFDMConfig.h"
#include "Assert.h"
#include "LeafNode.h"
#include "MatrixInputPort.h"
#include "MatrixOutputPort.h"
#include "RealInputPort.h"
#include "RealOutputPort.h"

namespace OpenFDM {

class DiscreteStateValueVector;
class ContinousStateValueVector;
class PortValueList;
class ContinousStateValueVector;

class Model : public LeafNode {
  OPENFDM_OBJECT(Model, LeafNode);
public:
  Model(const std::string& name);
  virtual ~Model();

  virtual void accept(NodeVisitor& visitor);

  // FIXME: May be we want to collapse all state values in one
  // argument? May be it is sufficient to have a const and non const version??
  // FIXME???
  // const TaskInfo& taskInfo
  virtual void update(DiscreteStateValueVector&, ContinousStateValueVector&,
                      const PortValueList&) const { }
  virtual void output(const DiscreteStateValueVector&,
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
  newMatrixInputPort(const std::string& name, const Size& size = Size(0, 0))
  { return MatrixInputPort(this, name, size); }
  RealOutputPort
  newRealOutputPort(const std::string& name)
  { return RealOutputPort(this, name); }
  RealInputPort
  newRealInputPort(const std::string& name)
  { return RealInputPort(this, name); }
};

} // namespace OpenFDM

#endif
