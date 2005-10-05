/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBReader_H
#define OpenFDM_JSBReader_H

#include <string>
#include <sstream>
#include <iostream>
#include <list>
#include <map>
#include <stack>
#include <locale>
using namespace std;

#include <simgear/xml/easyxml.hxx>

#include <OpenFDM/Expression.h>
#include <OpenFDM/AeroForce.h>
#include <OpenFDM/Vehicle.h>

namespace OpenFDM {

// Implements a SimGear SGProperty compatible 'path' to 'expression'
// mapping.
// It is used to map the value names occuring in JSBSim configuration files
// to OpenFDM::ObsoleteExpressions which will contain the values later.
// This is done by wrapping all map accesses where a key_type argument is given
// with a function which first simplyfys the given key and then performs the
// map operations with the simplyfied key instead of the original one.
// 
class PropertyMap
  : public std::map<std::string,Property> {
public:
  static std::string simplyfy(std::string path)
  {
    std::string::size_type idx;
    while ((idx = path.find("[0]")) != std::string::npos) {
      path.erase(idx, 3);
    }
    return path;
  }

  mapped_type&
  operator[](const key_type& key)
  { return std::map<std::string,Property>::operator[](simplyfy(key)); }

  size_type
  erase(const key_type& key)
  { return std::map<std::string,Property>::erase(simplyfy(key)); }

  iterator
  find(const key_type& key)
  { return std::map<std::string,Property>::find(simplyfy(key)); }

  const_iterator
  find(const key_type& key) const
  { return std::map<std::string,Property>::find(simplyfy(key)); }

  size_type
  count(const key_type& key) const
  { return std::map<std::string,Property>::count(simplyfy(key)); }

  iterator
  lower_bound(const key_type& key)
  { return std::map<std::string,Property>::lower_bound(simplyfy(key)); }

  const_iterator
  lower_bound(const key_type& key) const
  { return std::map<std::string,Property>::lower_bound(simplyfy(key)); }

  iterator
  upper_bound(const key_type& key)
  { return std::map<std::string,Property>::upper_bound(simplyfy(key)); }

  const_iterator
  upper_bound(const key_type& key) const
  { return std::map<std::string,Property>::upper_bound(simplyfy(key)); }

  pair<iterator,iterator>
  equal_range(const key_type& key)
  { return std::map<std::string,Property>::equal_range(simplyfy(key)); }

  pair<const_iterator,const_iterator>
  equal_range(const key_type& key) const
  { return std::map<std::string,Property>::equal_range(simplyfy(key)); }


  pair<iterator,bool>
  insert(const value_type& val)
  {
    value_type sval(simplyfy(val.first), val.second);
    return std::map<std::string,Property>::insert(sval);
  }

  iterator
  insert(iterator position, const value_type& val)
  {
    value_type sval(simplyfy(val.first), val.second);
    return std::map<std::string,Property>::insert(position, sval);
  }
};





class JSBReader
  : public XMLVisitor {
public:
  JSBReader(void);
  virtual ~JSBReader(void);

  /// The interface to easyxml
  virtual void startElement(const char *name, const XMLAttributes& atts);
  virtual void endElement(const char *name);
  virtual void data(const char *s, int length);
  virtual void warning(const char *message, int line, int column);

  /// Return a pointer to the resulting vehicle
  Vehicle* getVehicle(void) { return mVehicle; }

  /// Convert a JSBSim, property name like it can appear in config files
  /// into a flightgear property name. That is it strips the optional - sign
  /// and prepends the property with fdm/jsbsim/
  std::string propNameFromJSBSim(const std::string& jsbSymbol);

  /// Returns true in case the JSBSim property contains a minus
  bool propMinusFromJSBSim(const std::string& jsbSymbol);

  /// Returns the name of the output property given the fcs component's name
  std::string normalizeComponentName(const std::string& name);



  Property lookupJSBExpression(const std::string& name);

  void registerExpression(const std::string& name, Property expr);
  void registerJSBExpression(const std::string& name, Property expr);

  Property createAndScheduleInput(const std::string& name);

  Property addInputModel(const std::string& name, const std::string& propName,
                         real_type gain = 1);
  void addOutputModel(const Property& out, const std::string& name,
                      const std::string& propName, real_type gain = 1);

  Property addInverterModel(const std::string& name, Property& in);
  Property addAbsModel(const std::string& name, Property& in);

  void addFCSModel(Model* model);


  void parseMetrics(istream& data);
  void parseUndercarriage(istream& data);
  void parseFCSComponent(istream& data, const char* name, const char* type);
  TypedProperty<real_type> parseCoefficient(istream& data, const char* type);
  void parseTank(istream& data, const char* type, const char* number);
  void parseThruster(istream& data, const char* file);
  void parseEngine(istream& data, const char* file);

  Vector3 structToBody(const Vector3& v)
  {
    Vector3 cgoff = v - mCG;
    return convertFrom(uInch, Vector3(-cgoff(1), cgoff(2), -cgoff(3)));
  }

  void makeAeroprops(void);
  

private:
  enum ParseMode {
    PROPULSIONMode,
    FCSMode,
    AERODYNAMICSMode,
    UNKOWNMode
  };

  enum Axis {
    Lift,
    Drag,
    Side,
    Roll,
    Pitch,
    Yaw
  };

  enum FCSComponent {
    SummerComponent,
    DeadbandComponent,
    GradientComponent,
    SwitchComponent,
    KinematComponent,
    GainComponent,
    SchedGainComponent,
    AeroScaleComponent,
    IntegratorComponent,
    FilterComponent
  };

  typedef PropertyMap expression_table;

  expression_table mExpressionTable;
  Property mPrevousFCSOutput;

  ParseMode mParseMode;

  Axis mAxis;

  struct tagdata {
    tagdata(const std::string& n, const XMLAttributes& atts)
      : name(n), attributes(atts)
    {}
    std::string name;
    XMLAttributesDefault attributes;
    std::stringstream data;
  };

  std::stack<tagdata*> mElementStack;

  std::stack<shared_ptr<SumExpressionImpl> > mCurrentAxis;
  std::stack<shared_ptr<ProductExpressionImpl> > mCurrentAxisProd;

  shared_ptr<Vehicle> mVehicle;
  shared_ptr<AeroForce> mAeroForce;

  Vector3 mCG;
  unsigned mEngineNumber;
  unsigned mGearNumber;
};

} // namespace OpenFDM

#endif
