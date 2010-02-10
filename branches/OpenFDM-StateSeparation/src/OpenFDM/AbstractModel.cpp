/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "AbstractModel.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(AbstractModel, LeafNode)
  END_OPENFDM_OBJECT_DEF

AbstractModel::AbstractModel(const std::string& name) :
  LeafNode(name)
{
}

AbstractModel::~AbstractModel()
{
}

void
AbstractModel::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
AbstractModel::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

} // namespace OpenFDM
