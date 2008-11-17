/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DeadBand_H
#define OpenFDM_DeadBand_H

#include <string>

#include "UnaryModel.h"

namespace OpenFDM {

class DeadBand : public UnaryModel {
  OPENFDM_OBJECT(DeadBand, UnaryModel);
public:
  DeadBand(const std::string& name, const real_type& width = real_type(0));
  virtual ~DeadBand(void);

  ModelContext* newModelContext(PortValueList&) const;

  void output(const Matrix& inputValue, Matrix& outputValue) const;

  const real_type& getWidth(void) const;
  void setWidth(const real_type& width);

private:
  real_type mWidth;
};

} // namespace OpenFDM

#endif
