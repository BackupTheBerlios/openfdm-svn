/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Tank_H
#define OpenFDM_Tank_H

#include "Mass.h"

namespace OpenFDM {

class Tank : public Mass {
  OPENFDM_OBJECT(Tank, Mass);
public:
  Tank(const std::string& name);
  virtual ~Tank(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);
  virtual void update(const TaskInfo&);

  virtual void setDiscreteState(const StateStream& state);
  virtual void getDiscreteState(StateStream& state) const;

  /// get the actual content of the tank in kg
  const real_type& getContent(void) const;
  /// set the actual content of the tank in kg
  void setContent(const real_type& content);

  /// get the capacity of the tank in kg(FIXME:use m^3 and fiddle wiht density)
  const real_type& getCapacity(void) const;
  /// set the capacity of the tank in kg(FIXME:use m^3 and fiddle wiht density)
  void setCapacity(const real_type& capacity);

  /// return nonzero if the tank is empty
  const unsigned& getIsEmpty(void) const;

private:
  real_type mContent;
  real_type mNextContent;
  real_type mCapacity;
  unsigned mIsEmpty;
  RealPortHandle mInputPort;
};

} // namespace OpenFDM

#endif
