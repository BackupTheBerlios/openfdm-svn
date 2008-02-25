/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Input_H
#define OpenFDM_Input_H

#include "Model.h"

namespace OpenFDM {

class Input : public Model {
  OPENFDM_OBJECT(Input, Model);
public:
  Input(const std::string& name);
  virtual ~Input(void);

  /// Double dispatch helper for the system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  virtual const Input* toInput(void) const;
  virtual Input* toInput(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  class Callback : public WeakReferenced {
  public:
    virtual ~Callback();
    virtual real_type getValue() const = 0;
  };

  Callback* getCallback(void) const;
  void setCallback(Callback* callback);

  const real_type& getInputGain(void) const;
  void setInputGain(const real_type& inputGain);

  const std::string& getInputName(void) const;
  void setInputName(const std::string& inputName);

  const real_type& getOutputValue(void) const;

private:
  SharedPtr<Callback> mCallback;
  real_type mOutputValue;
  real_type mInputGain;
  std::string mInputName;
};

} // namespace OpenFDM

#endif
