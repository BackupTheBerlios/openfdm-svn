/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Model.h"

#include "ConstNodeVisitor.h"
#include "LogStream.h"
#include "ModelContext.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Model, LeafNode)
  END_OPENFDM_OBJECT_DEF

Model::Model(const std::string& name) :
  LeafNode(name)
{
}

Model::~Model()
{
}

void
Model::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Model::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

ModelContext*
Model::newModelContext(PortValueList& portValueList) const
{
  SharedPtr<ModelContext> context = new ModelContext(this);
  for (unsigned i = 0; i < getNumPorts(); ++i) {
    PortValue* portValue = portValueList.getPortValue(i);
    if (!portValue) {
      Log(Model, Error) << "No port value given for model \"" << getName()
                        << "\" and port \"" << getPort(i)->getName()
                        << "\"" << endl;
      return false;
    }
    context->setPortValue(*getPort(i), portValue);
  }
  if (!context->alloc()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return false;
  }
  return context.release();
}

} // namespace OpenFDM
