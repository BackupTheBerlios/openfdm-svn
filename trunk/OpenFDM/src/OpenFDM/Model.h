/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Model_H
#define OpenFDM_Model_H

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Property.h"
#include "Vector.h"

namespace OpenFDM {

class ModelGroup;

class Model
  : public Object {
public:
  Model(const std::string& name);
  virtual ~Model(void);

  virtual const ModelGroup* toModelGroup(void) const;
  virtual ModelGroup* toModelGroup(void);

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

  /// FIXME Hmm, may be different ...
  void evalFunction(real_type t, const Vector& v, Vector& out);
  /// Compute the jacobian
  /// The default implementation computes a numeric approximation by finite
  /// differences
  virtual void evalJacobian(real_type t, const Vector& state,
                            Matrix& jac, unsigned offset = 0u);

  virtual void setDiscreteState(const Vector& state, unsigned offset);
  virtual void getDiscreteState(Vector& state, unsigned offset) const;

  /// Return the number of continous states
  unsigned getNumContinousStates(void) const
  { return mNumContinousStates; }
  /// Return the number of discrete states
  unsigned getNumDiscreteStates(void) const
  { return mNumDiscreteStates; }
  /// Return if the outputs containe a direct dependency on an input
  bool getDirectFeedThrough(void) const
  { return mDirectFeedThrough; }

  /// Returns the systems name.
  const std::string& getName(void) const
  { return mName; }
  void setName(const std::string& name)
  { mName = name; }

  /// Returns the number of input properties.
  unsigned getNumInputPorts(void) const
  { return mInputPorts.size(); }

  /// Returns the name of the i-th input property.
  const std::string& getInputPortName(unsigned i) const;

  /// Sets the i-th input property.
  bool setInputPort(unsigned i, const Property& prop);

  /// Sets the input with the given name property.
  bool setInputPort(const std::string& name, const Property& prop);

  const Property& getInputPort(const std::string& name) const;
  Property& getInputPort(const std::string& name);

  /// Returns the i-th input property.
  const Property& getInputPort(unsigned i) const
  {
    OpenFDMAssert(i < mInputPorts.size());
    return mInputPorts[i].property;
  }
  /// Returns the i-th input property.
  Property& getInputPort(unsigned i)
  {
    OpenFDMAssert(i < mInputPorts.size());
    return mInputPorts[i].property;
  }

  unsigned getNumOutputPorts(void) const
  { return mOutputPorts.size(); }

  Property getOutputPort(unsigned i) const;
  const std::string& getOutputPortName(unsigned i) const;

  /// Returns the systems output property with the given name.
  Property getOutputPort(const std::string& name) const;

  bool dependsDirectOn(const Model* const model) const;

protected:
  void setNumContinousStates(unsigned numContinousStates);
  void setNumDiscreteStates(unsigned numDiscreteStates);
  void setDirectFeedThrough(bool directFeedThrough)
  { mDirectFeedThrough = directFeedThrough; }

  /// Sets the number of input properties.
  void setNumInputPorts(unsigned num);

  /// Sets the name of the i-th input property.
  void setInputPortName(unsigned i, const std::string& name);

  /// Gets called whenever an input property is changed.
  /// Lets a Model implementation catch up changes to the model.
  virtual void inputPortChanged(unsigned i);

  /// Sets the number of output properties.
  void setNumOutputPorts(unsigned num);

  /// Sets the name of the i-th output property.
  void setOutputPort(unsigned i, const std::string& name,
                     const Property& prop);

private:
  // Sets the parent model.
  // That is the one which is informed if the number of states changes.
  void setParent(ModelGroup* modelGroup);
  const ModelGroup* getParent(void) const { return mParentModel; }
  ModelGroup* getParent(void) { return mParentModel; }
  void adjustNumContinousStates(unsigned newCount, unsigned oldCount);
  void adjustNumDiscreteStates(unsigned newCount, unsigned oldCount);

  /// Holds the single input property.
  struct Port {
    Property    property;
    std::string name;
  };
  std::vector<Port> mInputPorts;
  managed_ptr<ModelGroup> mParentModel;
  std::string mName;
  unsigned mNumContinousStates;
  unsigned mNumDiscreteStates;
  bool mDirectFeedThrough;
  std::vector<Port> mOutputPorts;

  // FIXME
  friend class ModelGroup;
};

} // namespace OpenFDM

#endif
