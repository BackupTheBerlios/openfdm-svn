/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LegacyJSBSimReader_H
#define OpenFDM_LegacyJSBSimReader_H

#include <string>
#include <list>
#include <map>
#include <iosfwd>

#include <OpenFDM/AeroForce.h>
#include <OpenFDM/Expression.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/ReaderWriter.h>

namespace OpenFDM {

class XMLDocument;
class XMLElement;

class Summer;
class Product;

// Implements a SimGear SGProperty compatible 'path' to 'expression'
// mapping.
// It is used to map the value names occuring in JSBSim configuration files
// to OpenFDM::ObsoleteExpressions which will contain the values later.
// This is done by wrapping all map accesses where a key_type argument is given
// with a function which first simplyfys the given key and then performs the
// map operations with the simplyfied key instead of the original one.
// 
/// FIXME!!!!!!!!
class PropertyMap
  : public std::map<std::string,SharedPtr<Port> > {
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
  { return std::map<std::string,SharedPtr<Port> >::operator[](simplyfy(key)); }

  size_type
  erase(const key_type& key)
  { return std::map<std::string,SharedPtr<Port> >::erase(simplyfy(key)); }

  iterator
  find(const key_type& key)
  { return std::map<std::string,SharedPtr<Port> >::find(simplyfy(key)); }

  const_iterator
  find(const key_type& key) const
  { return std::map<std::string,SharedPtr<Port> >::find(simplyfy(key)); }

  size_type
  count(const key_type& key) const
  { return std::map<std::string,SharedPtr<Port> >::count(simplyfy(key)); }

  iterator
  lower_bound(const key_type& key)
  { return std::map<std::string,SharedPtr<Port> >::lower_bound(simplyfy(key)); }

  const_iterator
  lower_bound(const key_type& key) const
  { return std::map<std::string,SharedPtr<Port> >::lower_bound(simplyfy(key)); }

  iterator
  upper_bound(const key_type& key)
  { return std::map<std::string,SharedPtr<Port> >::upper_bound(simplyfy(key)); }

  const_iterator
  upper_bound(const key_type& key) const
  { return std::map<std::string,SharedPtr<Port> >::upper_bound(simplyfy(key)); }

  std::pair<iterator,iterator>
  equal_range(const key_type& key)
  { return std::map<std::string,SharedPtr<Port> >::equal_range(simplyfy(key)); }

  std::pair<const_iterator,const_iterator>
  equal_range(const key_type& key) const
  { return std::map<std::string,SharedPtr<Port> >::equal_range(simplyfy(key)); }


  std::pair<iterator,bool>
  insert(const value_type& val)
  {
    value_type sval(simplyfy(val.first), val.second);
    return std::map<std::string,SharedPtr<Port> >::insert(sval);
  }

  iterator
  insert(iterator position, const value_type& val)
  {
    value_type sval(simplyfy(val.first), val.second);
    return std::map<std::string,SharedPtr<Port> >::insert(position, sval);
  }
};


class LegacyJSBSimReader :
    public ReaderWriter {
public:
  LegacyJSBSimReader(void);
  virtual ~LegacyJSBSimReader(void);

  virtual void reset(void);

  /// Add a new path component to search for aircraft configurations
  /// no getter available
  void addAircraftPath(const std::string& path);

  /// Add a new path component to search for engine configurations
  /// no getter available
  void addEnginePath(const std::string& path);

  /// Load the aircraft given in acFileName
  /// Returns true on successfull load
  bool loadAircraft(const std::string& acFileName);

private:
  /// Locates a file from a given path list
  static bool openFile(const std::list<std::string>& paths,
                       const std::string& file, std::ifstream& fs);
  /// Takes a whole config file in a string and makes valid xml
  /// from JSBSim files
  static void fixupTEST(std::string& s);

  /// Convert a JSBSim, property name like it can appear in config files
  /// into a flightgear property name. That is it strips the optional - sign
  /// and prepends the property with fdm/jsbsim/
  static std::string propNameFromJSBSim(const std::string& jsbSymbol);

  /// Returns true in case the JSBSim property contains a minus
  static bool propMinusFromJSBSim(const std::string& jsbSymbol);

  /// Returns the name of the output property given the fcs component's name
  static std::string normalizeComponentName(const std::string& name);


  /// <FIXME> document and rethink
  Port* lookupJSBExpression(const std::string& name);

  void registerExpression(const std::string& name, Port* expr);
  void registerJSBExpression(const std::string& name, Port* expr);

  Port* createAndScheduleAeroProp(const std::string& name);
  Port* createAndScheduleInput(const std::string& name);

  Port* addInputModel(const std::string& name, const std::string& propName,
                      real_type gain = 1);
  void addOutputModel(Port* out, const std::string& name,
                      const std::string& propName, real_type gain = 1);

  Port* addInverterModel(const std::string& name, Port* in);
  Port* addAbsModel(const std::string& name, Port* in);
  Port* addConstModel(const std::string& name, real_type value);

  void addFCSModel(Model* model);

  Port* addMultiBodyToUnit(const std::string& name, Unit u, Port* in);
  Port* addMultiBodyFromUnit(const std::string& name, Unit u, Port* in);
  Port* addMultiBodyAbsModel(const std::string& name, Port* in);
  void addMultiBodyModel(Model* model);
  /// </FIXME> document and rethink


  /// converts the top dom like representation to an OpenFDM Vehicle
  bool convertDocument(const XMLDocument* jsbDoc);
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
  bool convertEngine(const std::string& data, const std::string& type,
                     const std::string& number);
  /// converts AERODYNAMICS elements
  bool convertAerodynamics(const XMLElement* aerodynamics);
  /// converts recursively AERODYNAMICS summands, factors and grooups
  bool convertAEROSummands(const XMLElement* aeroSummands,
                           Summer* sum, Product* prod);

  Port* convertCoefficient(const std::string& data, const std::string& type);



  /// List for the aircraft search path.
  std::list<std::string> mAircraftPath;
  /// List for the engine search path.
  std::list<std::string> mEnginePath;


  // For now just copies from the prevous try ...
  Vector3 structToBody(const Vector3& v)
  {
    Vector3 cgoff = v - mBodyReference;
    return convertFrom(uInch, Vector3(-cgoff(1), cgoff(2), -cgoff(3)));
  }
  SharedPtr<Port> mPrevousFCSOutput;
  PropertyMap mExpressionTable;
  SharedPtr<AeroForce> mAeroForce;
  Vector3 mBodyReference;
};

} // namespace OpenFDM

#endif
