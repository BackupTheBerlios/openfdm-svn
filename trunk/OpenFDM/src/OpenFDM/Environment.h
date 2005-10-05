/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Environment_H
#define OpenFDM_Environment_H

#include "Object.h"

namespace OpenFDM {

class Atmosphere;
class Gravity;
class Ground;
class Planet;
class Turbulence;
class Wind;

class EnvironmentObject;

class Environment :
    public Object {
public:
  Environment(void);
  virtual ~Environment(void);

  /** Set atmosphere model.
   */
  void setAtmosphere(Atmosphere* p);

  /** Get atmosphere model.
   */
  const Atmosphere* getAtmosphere(void) const
  { return mAtmosphere; }

  /** Set gravity model.
   */
  void setGravity(Gravity* g);

  /** Get gravity model.
   */
  const Gravity* getGravity(void) const
  { return mGravity; }

  /** Set ground callback.
   */
  void setGround(Ground* g);

  /** Get ground callback.
   */
  const Ground* getGround(void) const
  { return mGround; }

  /** Set planet callback.
   */
  void setPlanet(Planet* p);

  /** Get planet callback.
   */
  const Planet* getPlanet(void) const
  { return mPlanet; }

  /** Set turbulence model.
   */
  void setTurbulence(Turbulence* p);

  /** Get turbulence model.
   */
  const Turbulence* getTurbulence(void) const
  { return mTurbulence; }

  /** Set wind callback.
   */
  void setWind(Wind* p);

  /** Get wind callback.
   */
  const Wind* getWind(void) const
  { return mWind; }

private:
  void attachEnvironmentObject(EnvironmentObject* environmentObject);
  void detachEnvironmentObject(EnvironmentObject* environmentObject);

  shared_ptr<Atmosphere> mAtmosphere;
  shared_ptr<Gravity> mGravity;
  shared_ptr<Ground> mGround;
  shared_ptr<Planet> mPlanet;
  shared_ptr<Turbulence> mTurbulence;
  shared_ptr<Wind> mWind;
};

} // namespace OpenFDM

#endif
