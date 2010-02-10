/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MinModel_H
#define OpenFDM_MinModel_H

#include <string>

#include "SimpleDirectModel.h"

namespace OpenFDM {

class MinModel : public SimpleDirectModel {
  OPENFDM_OBJECT(MinModel, SimpleDirectModel);
public:
  MinModel(const std::string& name);
  virtual ~MinModel(void);

  virtual void output(Context& context) const;
  
  unsigned getNumMinInputs(void) const;
  void setNumMinInputs(unsigned num);
};

} // namespace OpenFDM

#endif
