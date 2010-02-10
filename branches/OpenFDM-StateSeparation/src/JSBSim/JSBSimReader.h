/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimReader_H
#define OpenFDM_JSBSimReader_H

#include <string>
#include <list>
#include <map>
#include <iosfwd>

#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/Table.h>
#include <OpenFDM/UnaryFunction.h>
#include <OpenFDM/BinaryFunction.h>

#include "JSBSimReaderBase.h"

namespace OpenFDM {

class XMLElement;

class Summer;
class Product;
class Port;

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
  bool convertSystem(const XMLElement* system);
  bool attachWheel(const XMLElement* wheelElem, const std::string& name,
                   const std::string& numStr, RigidBody* parent);
  bool convertGroundReactionsElem(const XMLElement* gr);
  bool convertPropulsion(const XMLElement* prop);
  bool convertTank(const XMLElement* tElem, const std::string& number);
  bool convertThruster(const XMLElement* tElem, const std::string& number);
  bool convertEngine(const XMLElement* engine, const std::string& number);
  bool convertTurbine(const XMLElement* turbine, const std::string& number,
                      const Vector3& pos, const Quaternion& orientation,
                      const Port* thrusterDriver);
  bool convertTurboProp(const XMLElement* turbine, const std::string& number,
                        const Vector3& pos, const Quaternion& orientation,
                        const Port* thrusterDriver);
  bool convertElectric(const XMLElement* turbine, const std::string& number,
                       const Port* thrusterDriver);
  bool convertPiston(const XMLElement* turbine, const std::string& number,
                     const Port* thrusterDriver);
  bool convertAerodynamics(const XMLElement* aero);
  bool convertFunction(const XMLElement* function, Summer* sum);

  bool connectUnaryFunctionInput(const std::string& name,
                                 UnaryFunction::Type type,
                                 const XMLElement* element,
                                 const Port* port, Group* parentGroup);
  bool connectBinaryFunctionInput(const std::string& name,
                                  BinaryFunction::Type type,
                                  const XMLElement* element,
                                  const Port* port, Group* parentGroup);

  bool connectFunctionInput(const XMLElement* element, const Port* port,
                            Group* parentGroup);

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
