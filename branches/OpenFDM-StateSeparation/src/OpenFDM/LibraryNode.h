/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LibraryNode_H
#define OpenFDM_LibraryNode_H

#include <string>
#include "LibraryModel.h"
#include "Node.h"
#include "SharedPtr.h"

namespace OpenFDM {

class LibraryNode : public Node {
  OPENFDM_OBJECT(LibraryNode, Node);
public:
  LibraryNode(const std::string& name, LibraryModel* libraryModel = 0);
  virtual ~LibraryNode();

  virtual void accept(NodeVisitor& visitor);

  // FIXME: Hmm, how do we map ports??
  // May be the Node just gets virtuals for ports???
  // May be changing ports means informing the parent about that???

  SharedPtr<LibraryModel> getLibraryModel();
  SharedPtr<const LibraryModel> getLibraryModel() const;
  void setLibraryModel(LibraryModel* libraryModel);

private:
  SharedPtr<LibraryModel> mLibraryModel;
};

} // namespace OpenFDM

#endif
