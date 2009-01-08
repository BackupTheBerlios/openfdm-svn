/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MaxModel_H
#define OpenFDM_MaxModel_H

#include <string>

#include "SimpleDirectModel.h"

namespace OpenFDM {

class MaxModel : public SimpleDirectModel {
  OPENFDM_OBJECT(MaxModel, SimpleDirectModel);
public:
  MaxModel(const std::string& name);
  virtual ~MaxModel(void);
  
  virtual void output(Context& context) const;

  unsigned getNumMaxInputs(void) const;
  void setNumMaxInputs(unsigned num);
};

} // namespace OpenFDM

#endif
