/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimReaderBase_H
#define OpenFDM_JSBSimReaderBase_H

#include <map>
#include <string>
#include <cstring>
#include <list>
#include <map>
#include <iosfwd>

#include <OpenFDM/Assert.h>
#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/BreakPointLookup.h>
#include <OpenFDM/Table.h>
#include <OpenFDM/Model.h>
#include <OpenFDM/GroupInput.h>
#include <OpenFDM/GroupOutput.h>
#include <OpenFDM/Port.h>

#include <OpenFDM/XML/XMLReader.h>
#include <OpenFDM/XML/ContentHandler.h>
#include <OpenFDM/XML/ErrorHandler.h>
#include <OpenFDM/XML/Attributes.h>

#include <OpenFDM/XML/EasyXMLReader.h> // FIXME

#include "JSBSimPropertyManager.h"
#include "JSBSimAerodynamic.h"
#include "XMLReader.h"

namespace OpenFDM {

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

  /// Add a new path component to search for system configurations
  /// no getter available
  void addSystemPath(const std::string& path);

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

  bool connectJSBExpression(const std::string& name, const Port*,
                            bool recheckAeroProp = true);

  std::string canonicalJSBProperty(std::string name);

  bool registerExpression(const std::string& name, const Port* expr);
  bool registerJSBExpression(const std::string& name, const Port* expr);

  bool provideSubstitutes();
  bool provideSubstitute(const std::string&);
  bool connect();

  const Port* addInputModel(const std::string& name,
                            const std::string& propName, real_type gain = 1);
  void addOutputModel(const Port* out, const std::string& name,
                      const std::string& propName, real_type gain = 1);
  void addOutputModel(const std::string& inputProp, const std::string& name,
                      const std::string& propName, real_type gain = 1);

  const Port* addInverterModel(const std::string& name, const Port* in);
  const Port* addInverterModel(const std::string& name,
                               const std::string& inProp);
  const Port* addAbsModel(const std::string& name, const Port* in);
  const Port* addAbsModel(const std::string& name, const std::string& inProp);
  const Port* addConstModel(const std::string& name, real_type value,
                            const NodePath& path = NodePath());
  const Port* addToUnit(const std::string& name, Unit u, const Port* in);
  const Port* addToUnit(const std::string& name, Unit u,
                        const std::string& inProp);
  const Port* addFromUnit(const std::string& name, Unit u, const Port* in);
  const Port* addFromUnit(const std::string& name, Unit u,
                          const std::string& inProp);

  SharedPtr<Group> getGroup(const Port* in);
  void addFCSModel(Node* model);

  const Port* addMultiBodyConstModel(const std::string& name, real_type value);
  void addMultiBodyModel(Node* model);
  /// </FIXME> document and rethink
  const Port* getTablePrelookup(const std::string& name,
                                const std::string& inputProperty,
                                const BreakPointVector& tl);
  /// List for the aircraft search path.
  std::list<std::string> mAircraftPath;
  /// List for the engine search path.
  std::list<std::string> mEnginePath;
  /// List for the system search path.
  std::list<std::string> mSystemPath;

  SharedPtr<Group> mTopLevelGroup;
  JSBSimPropertyManager mPropertyManager;
  SharedPtr<JSBSimAerodynamic> mAeroForce;
  SharedPtr<RigidBody> mTopLevelBody;
  struct BreakPointLookupEntry {
    BreakPointLookupEntry(const std::string& pname = std::string(), BreakPointLookup* tl = 0) :
      propertyName(pname), lookup(tl) {}
    std::string propertyName;
    SharedPtr<BreakPointLookup> lookup;
  };
  std::vector<BreakPointLookupEntry> mBreakPointVectors;

  // For now just copies from the prevous try ...
  Vector3 structToBody(const Vector3& v)
  {
    Vector3 cgoff = v - mBodyReference;
    return Unit::inch().convertFrom(Vector3(-cgoff(0), cgoff(1), -cgoff(2)));
  }
  Vector3 mBodyReference;
};

} // namespace OpenFDM

#endif
