/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LegacyJSBSimReader_H
#define OpenFDM_LegacyJSBSimReader_H

#include <string>
#include <list>
#include <map>
#include <iosfwd>

#include <OpenFDM/AeroForce.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/Table.h>

#include "JSBSimReaderBase.h"

namespace OpenFDM {

class XMLElement;

class Summer;
class Product;

class LegacyJSBSimReader : public JSBSimReaderBase {
public:
  LegacyJSBSimReader(void);
  virtual ~LegacyJSBSimReader(void);

  virtual void reset(void);

  /// Load the aircraft given in acFileName
  /// Returns true on successfull load
  bool loadAircraft(const std::string& acFileName);

private:
  /// Takes a whole config file in a string and makes valid xml
  /// from JSBSim files
  static void fixupTEST(std::string& s);

  /// converts the top dom like representation to an OpenFDM Vehicle
  bool convertDocument(const XMLElement* topElement);
  /// converts the METRICS data
  bool convertMetrics(const std::string& data);
  /// Helper for convertUndercarriage
  void attachWheel(const std::string& name, const Vector3& pos,
                   const std::string& brake,
                   const std::string& numStr, real_type wheelDiam,
                   real_type tireSpring, real_type tireDamp,
                   RigidBody* parent);
  /// converts the UNDERCARRIAGE data
  bool convertUndercarriage(const std::string& data);
  /// converts the FLIGHT_CONTROL or AUTOPILOT elements
  bool convertFCSList(const XMLElement* fcsElement);
  /// converts a single FCS component
  bool convertFCSComponent(const std::string& type, const std::string& name,
                           const std::string& data);
  /// converts the propulsion elements
  bool convertPropulsion(const XMLElement* pElem);
  /// converts FG_TANK data
  bool convertTank(const std::string& data, const std::string& type,
                   const std::string& number);
  /// converts FG_THRUSTER data
  bool convertThruster(const std::string& data, const std::string& type,
                       const std::string& number);
  /// converts FG_ENGINE data
  bool convertEngine(const XMLElement* engine, const std::string& number);
  /// converts FG_TURBINE data
  bool convertTurbine(const XMLElement* turbine, const std::string& number,
                      const Vector3& pos, const Quaternion& orientation,
                      Port* thrusterDriver);
  /// converts FG_PISTON data
  bool convertPiston(const XMLElement* turbine, const std::string& number,
                      Port* thrusterDriver);
  /// converts FG_ELECTRIC data
  bool convertElectric(const XMLElement* turbine, const std::string& number,
                      Port* thrusterDriver);
  /// converts AERODYNAMICS elements
  bool convertAerodynamics(const XMLElement* aerodynamics);
  /// converts recursively AERODYNAMICS summands, factors and grooups
  bool convertAEROSummands(const XMLElement* aeroSummands,
                           Summer* sum, Product* prod);

  Port* convertCoefficient(const std::string& data, const std::string& type,
                           const std::string& name);

  SharedPtr<Port> mPrevousFCSOutput;
};

} // namespace OpenFDM

#endif
