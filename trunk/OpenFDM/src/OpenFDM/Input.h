/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Input_H
#define OpenFDM_Input_H

#include "Model.h"

namespace OpenFDM {

class Input : public Model {
public:
  Input(const std::string& name);
  virtual ~Input(void);
  
  virtual bool init(void);
  virtual void output(void);

  const real_type& getInputValue(void) const;
  void setInputValue(const real_type& value);

  const real_type& getInputGain(void) const;
  void setInputGain(const real_type& inputGain);

  const std::string& getInputName(void) const;
  void setInputName(const std::string& inputName);

  const real_type& getOutputValue(void) const;

private:
  real_type mInputValue;
  real_type mOutputValue;
  real_type mInputGain;
  std::string mInputName;
};

class Output : public Model {
public:
  Output(const std::string& name);
  virtual ~Output(void);
  
  virtual bool init(void);
  virtual void output(void);

  const real_type& getValue(void) const;

  const real_type& getOutputGain(void) const;
  void setOutputGain(const real_type& outputGain);

  const std::string& getOutputName(void) const;
  void setOutputName(const std::string& outputName);

private:
  real_type mValue;
  real_type mOutputGain;
  std::string mOutputName;
};

} // namespace OpenFDM

#endif
