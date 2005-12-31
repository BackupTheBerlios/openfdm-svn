/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelGroup_H
#define OpenFDM_ModelGroup_H

#include <vector>

#include "Object.h"
#include "Model.h"
#include "Property.h"
#include "Vector.h"

namespace OpenFDM {

class Joint;

class ModelGroup
  : public Model {
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
  unsigned addModel(Model* model);
  void removeModel(Model* model);

  /// Called on each system initialization.
  virtual bool init(void);
  /// Called when the outputs need to be prepared for the next step.
  /// Note that this is called *before* update() is called.
  virtual void output(const TaskInfo& taskInfo);
  /// Called whenever discrete states need to be updated.
  virtual void update(const TaskInfo& taskInfo);

  virtual void setState(const StateStream& state);
  virtual void getState(StateStream& state) const;
  virtual void getStateDeriv(StateStream& stateDeriv);

  virtual void setDiscreteState(const StateStream& state);
  virtual void getDiscreteState(StateStream& state) const;

  /// make them public
  using Model::setNumInputPorts;
  using Model::setNumOutputPorts;

protected: // FIXME
// private:
  typedef std::vector<SharedPtr<Model> > ModelList;

  /// Helper functions to sort the models according their dependencies
  static bool dependsOn(Port* inputPort, Model* model);
  /// return true if interact1 depends on interact2 which means that
  /// interact1 is higher in the tree of multibody models
  static bool dependsOnMultiBody(Joint* joint1, Joint* joint2);

  bool appendModel(const Model* firstModel, SharedPtr<Model> model,
                   ModelList& newList);
  /// Sorts the models depending their dependencies
  bool sortModels(void);

  /// The List of models contained in this group.
  ModelList mModels;
};

} // namespace OpenFDM

#endif
