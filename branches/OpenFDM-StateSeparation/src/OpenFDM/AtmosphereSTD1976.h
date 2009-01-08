/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AtmosphereSTD1976_H
#define OpenFDM_AtmosphereSTD1976_H

#include <map>

#include "Types.h"
#include "Object.h"
#include "AbstractAtmosphere.h"

namespace OpenFDM {

// http://www.pdas.com/atmos.htm
// http://nssdc.gsfc.nasa.gov/space/model/atmos/atmos_index.html
// ftp://nssdcftp.gsfc.nasa.gov/models/

class AtmosphereSTD1976 : public AbstractAtmosphere {
public:
  AtmosphereSTD1976(void);
  virtual ~AtmosphereSTD1976(void);

  // Sea level pressure, defaults to 101325 N/m2
  const real_type& getSeaLevelPressure() const
  { return mSlPressure; }
  void setSeaLevelPressure(const real_type& pressure)
  { mSlPressure = pressure; }

  // Sea level temperature, defaults to 288.15 K
  const real_type& getSeaLevelTemperature() const
  { return mSlTemperature; }
  void setSeaLevelTemperature(const real_type& temperature)
  { mSlTemperature = temperature; }

  // Get the atmosphere data for a given height.
  virtual AtmosphereData getData(const real_type&, const real_type& alt) const;
private:

  static const real_type mHydrostaticConstant;
  static const real_type mEarthRadius;

  struct TableData {
    TableData(void) : t(0.0), p(0.0), g(0.0) {}
    TableData(real_type t_, real_type p_, real_type g_) : t(t_), p(p_), g(g_){}
    real_type t;
    real_type p;
    real_type g;
  };

  typedef std::map<real_type, TableData> Table;

  // Should be static, but lacking a good initializer for the map ...
  Table mTable;

  // Sea level pressure
  real_type mSlPressure;
  // Sea level temperature
  real_type mSlTemperature;
};

} // namespace OpenFDM

#endif
