/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AtmosphereSTD1976_H
#define OpenFDM_AtmosphereSTD1976_H

#include <map>

#include "Types.h"
#include "Object.h"
#include "Atmosphere.h"

namespace OpenFDM {

// http://www.pdas.com/atmos.htm
// http://nssdc.gsfc.nasa.gov/space/model/atmos/atmos_index.html
// ftp://nssdcftp.gsfc.nasa.gov/models/

class AtmosphereSTD1976
  : public Atmosphere {
public:
  AtmosphereSTD1976(void);
  virtual ~AtmosphereSTD1976(void);

  // Get the atmosphere data for a given height.
  virtual AtmosphereData getData(real_type alt) const;
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
};

} // namespace OpenFDM

#endif
