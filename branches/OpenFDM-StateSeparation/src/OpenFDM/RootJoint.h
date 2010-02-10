/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RootJoint_H
#define OpenFDM_RootJoint_H

#include <string>
#include "Joint.h"

namespace OpenFDM {

class RootJoint : public Joint {
  OPENFDM_OBJECT(RootJoint, Joint);
public:
  RootJoint(const std::string& name);
  virtual ~RootJoint();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;
};

} // namespace OpenFDM

#endif
