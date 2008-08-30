/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Model.h"

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
  visitor.apply(*this);
}

} // namespace OpenFDM
