/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Gain_H
#define OpenFDM_Gain_H

#include <string>

#include "UnaryModel.h"
#include "ModelContext.h"

namespace OpenFDM {

class Gain : public UnaryModel {
  OPENFDM_OBJECT(Gain, UnaryModel);
public:
  Gain(const std::string& name, const real_type& gain = real_type(1));
  virtual ~Gain(void);

  ModelContext* newModelContext(PortValueList&) const;

  void output(const Matrix& inputValue, Matrix& outputValue) const;

  const real_type& getGain(void) const;
  void setGain(const real_type& gain);

private:
  real_type mGain;
};

} // namespace OpenFDM

#endif
