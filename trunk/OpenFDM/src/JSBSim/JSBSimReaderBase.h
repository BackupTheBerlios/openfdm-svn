/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimReaderBase_H
#define OpenFDM_JSBSimReaderBase_H

#include <string>
#include <list>
#include <map>
#include <iosfwd>

#include <OpenFDM/AeroForce.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/Table.h>

#include <OpenFDM/XML/XMLReader.h>
#include <OpenFDM/XML/ContentHandler.h>
#include <OpenFDM/XML/ErrorHandler.h>
#include <OpenFDM/XML/Attributes.h>

#include <OpenFDM/XML/EasyXMLReader.h> // FIXME

#include "XMLReader.h"

namespace OpenFDM {

class Element;

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

class JSBSimReaderBase : public ReaderWriter {
public:
  JSBSimReaderBase(void);
  virtual ~JSBSimReaderBase(void);

  virtual void reset(void);

  /// Add a new path component to search for aircraft configurations
  /// no getter available
  void addAircraftPath(const std::string& path);

  /// Add a new path component to search for engine configurations
  /// no getter available
  void addEnginePath(const std::string& path);

protected:
  /// Locates a file from a given path list
  static bool openFile(const std::list<std::string>& paths,
                       const std::string& file, std::ifstream& fs);
  static SharedPtr<XMLElement> parseXMLStream(std::istream& stream);

  /// Convert a JSBSim, property name like it can appear in config files
  /// into a flightgear property name. That is it strips the optional - sign
  /// and prepends the property with fdm/jsbsim/
  static std::string propNameFromJSBSim(const std::string& jsbSymbol);

  /// Returns true in case the JSBSim property contains a minus
  static bool propMinusFromJSBSim(const std::string& jsbSymbol);

  /// Returns the name of the output property given the fcs component's name
  static std::string normalizeComponentName(const std::string& name);


  /// <FIXME> document and rethink
  Port* lookupJSBExpression(const std::string& name,
                            bool recheckAeroProp = true);

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
  Port* addToUnit(const std::string& name, Unit u, Port* in);
  Port* addFromUnit(const std::string& name, Unit u, Port* in);

  void addFCSModel(Model* model);

  Port* addMultiBodyToUnit(const std::string& name, Unit u, Port* in);
  Port* addMultiBodyFromUnit(const std::string& name, Unit u, Port* in);
  Port* addMultiBodyAbsModel(const std::string& name, Port* in);
  Port* addMultiBodyConstModel(const std::string& name, real_type value);
  void addMultiBodyModel(Model* model);
  /// </FIXME> document and rethink
  Port* getTablePrelookup(const std::string& name, Port* in, const TableLookup& tl);
  /// List for the aircraft search path.
  std::list<std::string> mAircraftPath;
  /// List for the engine search path.
  std::list<std::string> mEnginePath;

  PropertyMap mExpressionTable;
  SharedPtr<AeroForce> mAeroForce;
  std::vector<SharedPtr<TablePreLookup> > mTableLookups;

  // For now just copies from the prevous try ...
  Vector3 structToBody(const Vector3& v)
  {
    Vector3 cgoff = v - mBodyReference;
    return convertFrom(uInch, Vector3(-cgoff(1), cgoff(2), -cgoff(3)));
  }
  Vector3 mBodyReference;
};

} // namespace OpenFDM

#endif
