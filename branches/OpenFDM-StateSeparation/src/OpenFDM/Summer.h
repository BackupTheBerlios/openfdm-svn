/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Summer_H
#define OpenFDM_Summer_H

#include <string>
#include "SimpleDirectModel.h"

namespace OpenFDM {

class Summer : public SimpleDirectModel {
  OPENFDM_OBJECT(Summer, SimpleDirectModel);
public:
  Summer(const std::string& name);
  virtual ~Summer(void);
  
  virtual void output(Context& context) const;

  unsigned getNumSummands(void) const;
  void setNumSummands(unsigned num);

  enum Sign { Plus, Minus };
  void setInputSign(unsigned num, Sign sign);
  Sign getInputSign(unsigned num) const;

private:
  std::vector<Sign> mSigns;
};

} // namespace OpenFDM

#endif
