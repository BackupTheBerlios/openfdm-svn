/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_StateValue_H
#define OpenFDM_StateValue_H

#include "Referenced.h"

namespace OpenFDM {

class StateValue : public Referenced {
public:

  static void destroy(StateValue* stateValue)
  { delete stateValue; }

protected:
  virtual ~StateValue();
};

} // namespace OpenFDM

#endif
