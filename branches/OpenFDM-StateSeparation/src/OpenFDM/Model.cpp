/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Model.h"

#include "ConstNodeVisitor.h"
#include "LogStream.h"
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

} // namespace OpenFDM
