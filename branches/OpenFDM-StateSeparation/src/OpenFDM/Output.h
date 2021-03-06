/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Output_H
#define OpenFDM_Output_H

#include "Model.h"

namespace OpenFDM {

class Output : public Model {
  OPENFDM_OBJECT(Output, Model);
public:
  class Callback : public WeakReferenced {
  public:
    virtual ~Callback();
    virtual void setValue(real_type value) = 0;
  };

  Output(const std::string& name, Callback* callback = 0);
  virtual ~Output(void);
  
  /// Double dispatch helper for the system visitor
  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  virtual void output(const Task&,const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;

  Callback* getCallback(void) const;
  void setCallback(Callback* callback);

  const real_type& getOutputGain(void) const;
  void setOutputGain(const real_type& outputGain);

  const std::string& getOutputName(void) const;
  void setOutputName(const std::string& outputName);

private:
  RealInputPort mInputPort;
  SharedPtr<Callback> mCallback;
  real_type mOutputGain;
  std::string mOutputName;
};

} // namespace OpenFDM

#endif
