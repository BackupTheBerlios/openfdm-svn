/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimReader_H
#define OpenFDM_JSBSimReader_H

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
class Port;
class PortProvider;

class JSBSimReader : public JSBSimReaderBase {
public:
  JSBSimReader(void);
  virtual ~JSBSimReader(void);

  virtual void reset(void);

  /// Load the aircraft given in acFileName
  /// Returns true on successfull load
  bool loadAircraft(const std::string& acFileName);

private:
  bool convertDocument(const XMLElement* jsbDoc);
  bool convertMetrics(const XMLElement* metricsElement);
  bool convertMassBalance(const XMLElement* massBalanceElement);
  bool convertFCSList(const XMLElement* fcsElem);
  bool convertFCSComponent(const XMLElement* fcsComponent);
  bool attachWheel(const XMLElement* wheelElem, const std::string& name,
                   const std::string& numStr, RigidBody* parent,
                   const Vector3& parentDesignPos);
  bool convertGroundReactionsElem(const XMLElement* gr);
  bool convertPropulsion(const XMLElement* prop);
  bool convertTank(const XMLElement* tElem, const std::string& number);
  bool convertThruster(const XMLElement* tElem, const std::string& number);
  bool convertEngine(const XMLElement* engine, const std::string& number);
  bool convertTurbine(const XMLElement* turbine, const std::string& number,
                      const Vector3& pos, const Quaternion& orientation,
                      PortProvider* thrusterDriver);
  bool convertAerodynamics(const XMLElement* aero);
  bool convertFunction(const XMLElement* function, Summer* sum);
  std::list<PortProvider*> readFunctionInputs(const XMLElement* operationTag,
                                      const std::string& name);

  unsigned getNumTableDims(const XMLElement* tableElem);
  bool readTable1D(const XMLElement* tableElem,
                   TableData<1>& data, BreakPointVector& lookup);
  bool readTable2D(const XMLElement* tableElem,
                   TableData<2>& data, BreakPointVector lookup[2]);
  bool readTable3D(const XMLElement* tableElem,
                   TableData<3>& data, BreakPointVector lookup[3]);
};

} // namespace OpenFDM

#endif
