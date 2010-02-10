/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ContinousStateValue_H
#define OpenFDM_ContinousStateValue_H

#include "StateStream.h"
#include "StateValue.h"

namespace OpenFDM {

class ContinousStateValue : public StateValue {
public:
  virtual void setValue(const StateStream& stateStream) = 0;
  virtual void getValue(StateStream& stateStream) const = 0;
  virtual LinAlg::size_type getNumStates() const = 0;

protected:
  virtual ~ContinousStateValue();
};

} // namespace OpenFDM

#endif
