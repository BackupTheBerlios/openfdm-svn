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

class ModelGroup
  : public Model {
public:
  ModelGroup(const std::string& name);
  virtual ~ModelGroup(void);

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
  virtual void output(void);
  /// Called whenever discrete states need to be updated.
  virtual void update(real_type dt);

  virtual void setState(real_type t, const Vector& state, unsigned offset);
  virtual void getState(Vector& state, unsigned offset) const;
  virtual void getStateDeriv(Vector& stateDeriv, unsigned offset);

  virtual void setDiscreteState(const Vector& state, unsigned offset);
  virtual void getDiscreteState(Vector& state, unsigned offset) const;

private:
  typedef std::vector<shared_ptr<Model> > ModelList;

  /// Helper function to sort the models according their dependencies
  bool appendDependecies(const Model* firstModel, Model* model,
                         ModelList& newList);
  /// Sorts the models depending their dependencies
  bool sortModels(void);

  /// The List of models contained in this group.
  ModelList mModels;
};

} // namespace OpenFDM

#endif
