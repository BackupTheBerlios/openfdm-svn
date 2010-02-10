/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DeadBand_H
#define OpenFDM_DeadBand_H

#include <string>

#include "SimpleDirectModel.h"

namespace OpenFDM {

class DeadBand : public SimpleDirectModel {
  OPENFDM_OBJECT(DeadBand, SimpleDirectModel);
public:
  DeadBand(const std::string& name, const real_type& width = real_type(0));
  virtual ~DeadBand(void);

  void output(Context& context) const;

  const real_type& getWidth(void) const;
  void setWidth(const real_type& width);

private:
  real_type mWidth;
};

} // namespace OpenFDM

#endif
