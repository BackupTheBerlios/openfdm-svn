/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Saturation_H
#define OpenFDM_Saturation_H

#include <string>

#include "Matrix.h"
#include "SimpleDirectModel.h"

namespace OpenFDM {

class Saturation : public SimpleDirectModel {
  OPENFDM_OBJECT(Saturation, SimpleDirectModel);
public:
  Saturation(const std::string& name);
  virtual ~Saturation(void);
  
  void output(Context& context) const;

  const Matrix& getMinSaturation(void) const;
  void setMinSaturation(const Matrix& minSaturation);
  const Matrix& getMaxSaturation(void) const;
  void setMaxSaturation(const Matrix& maxSaturation);

private:
  Matrix mMaxSaturation;
  Matrix mMinSaturation;
};

} // namespace OpenFDM

#endif
