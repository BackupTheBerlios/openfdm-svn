/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractModel_H
#define OpenFDM_AbstractModel_H

#include <string>

#include "OpenFDMConfig.h"
#include "LeafNode.h"

namespace OpenFDM {

class AbstractModelContext;
class PortValueList;

class AbstractModel : public LeafNode {
  OPENFDM_OBJECT(AbstractModel, LeafNode);
public:
  AbstractModel(const std::string& name);
  virtual ~AbstractModel();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual AbstractModelContext* newModelContext(PortValueList&) const = 0;
};

} // namespace OpenFDM

#endif
