/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelGroup_H
#define OpenFDM_ModelGroup_H

#include <vector>

#include "Object.h"
#include "Model.h"
#include "Vector.h"

namespace OpenFDM {

class Joint;

class ModelGroup : public Model {
  OPENFDM_OBJECT(ModelGroup, Model);
public:
  ModelGroup(const std::string& name);
  virtual ~ModelGroup(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  void traverse(ModelVisitor& visitor)
  {
    for (ModelList::iterator it = mModels.begin(); it != mModels.end(); ++it)
      (*it)->accept(visitor);
  }


  virtual const ModelGroup* toModelGroup(void) const;
  virtual ModelGroup* toModelGroup(void);

  // FIXME: may be more name oriented, since this index will
  // change past scheduling
  unsigned getNumModels(void) const { return mModels.size(); }
  const Model* getModel(unsigned idx) const;
  Model* getModel(unsigned idx);
  const Model* getModel(const std::string& name) const;
  Model* getModel(const std::string& name);
  unsigned getModelIndex(const std::string& name) const;
  unsigned getModelIndex(const Model* model) const;
  unsigned addModel(Model* model);
  void removeModel(Model* model);

  /// make them public
  using Model::setNumInputPorts;
  using Model::setNumOutputPorts;

protected: // FIXME
// private:
  typedef std::vector<SharedPtr<Model> > ModelList;

  /// The List of models contained in this group.
  ModelList mModels;
};

} // namespace OpenFDM

#endif
