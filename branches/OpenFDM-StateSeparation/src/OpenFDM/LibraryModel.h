/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LibraryModel_H
#define OpenFDM_LibraryModel_H

#include <string>
#include <vector>
#include "Object.h"
#include "Node.h"
#include "SharedPtr.h"

namespace OpenFDM {

// Group |- LibraryNode
//       |- 

class LibraryNode;

class LibraryModel : public Object {
  OPENFDM_OBJECT(LibraryModel, Object);
public:
  LibraryModel(const std::string& name, Node* node = 0);
  virtual ~LibraryModel();

  unsigned getNumParentNodes() const;
  
  WeakPtr<LibraryNode> getParent(unsigned i);
  WeakPtr<const LibraryNode> getParent(unsigned i) const;

  SharedPtr<Node> getNode();
  SharedPtr<const Node> getNode() const;
  void setNode(Node* node);

private:
  SharedPtr<Node> mNode;
  std::vector<WeakPtr<LibraryNode> > mParentNodes;
};

} // namespace OpenFDM

#endif
