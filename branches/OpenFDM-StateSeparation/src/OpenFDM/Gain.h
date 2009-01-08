/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Gain_H
#define OpenFDM_Gain_H

#include <string>

#include "SimpleDirectModel.h"
#include "ModelContext.h"

namespace OpenFDM {

class Gain : public SimpleDirectModel {
  OPENFDM_OBJECT(Gain, SimpleDirectModel);
public:
  Gain(const std::string& name, const real_type& gain = real_type(1));
  virtual ~Gain(void);

  void output(Context& context) const;

  const real_type& getGain(void) const;
  void setGain(const real_type& gain);

private:
  real_type mGain;
};

} // namespace OpenFDM

#endif
