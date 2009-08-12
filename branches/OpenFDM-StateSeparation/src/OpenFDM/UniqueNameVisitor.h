/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_UniqueNameVisitor_H
#define OpenFDM_UniqueNameVisitor_H

#include <set>
#include <string>
#include <sstream>

#include "Group.h"
#include "NodeVisitor.h"

namespace OpenFDM {

class UniqueNameVisitor : public NodeVisitor {
public:
  virtual ~UniqueNameVisitor(void)
  { }
  virtual void apply(Group& group)
  {
    group.traverse(*this);

    typedef std::set<std::string> NameSet;
    NameSet nameSet;
    for (unsigned i = 0; i < group.getNumChildren(); ++i) {
      Node* node = group.getChild(i);
      std::string name = node->getName();
      unsigned counter = 0;
      while (nameSet.find(node->getName()) != nameSet.end()) {
        std::stringstream ss;
        ss << name << ++counter;
        node->setName(ss.str());
      }
      nameSet.insert(node->getName());
    }
  }
};

} // namespace OpenFDM

#endif
