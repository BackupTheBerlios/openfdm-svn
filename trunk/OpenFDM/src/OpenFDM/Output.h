/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Output_H
#define OpenFDM_Output_H

#include "Model.h"

namespace OpenFDM {

class Output : public Model {
  OPENFDM_OBJECT(Output, Model);
public:
  Output(const std::string& name);
  virtual ~Output(void);
  
  virtual const Output* toOutput(void) const;
  virtual Output* toOutput(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  class Callback : public WeakReferenced {
  public:
    virtual ~Callback();
    virtual void setValue(real_type value) = 0;
  };

  Callback* getCallback(void) const;
  void setCallback(Callback* callback);

  const real_type& getValue(void) const;

  const real_type& getOutputGain(void) const;
  void setOutputGain(const real_type& outputGain);

  const std::string& getOutputName(void) const;
  void setOutputName(const std::string& outputName);

private:
  RealPortHandle mInputPort;
  SharedPtr<Callback> mCallback;

  real_type mValue;
  real_type mOutputGain;
  std::string mOutputName;
};

} // namespace OpenFDM

#endif
