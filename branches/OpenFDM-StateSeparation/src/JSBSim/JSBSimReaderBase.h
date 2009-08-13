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

#include "JSBSimAerodynamic.h"
#include "XMLReader.h"

namespace OpenFDM {

class Summer;
class Product;

class PortSet {
  struct PathPort {
    NodePath modelPath;
    SharedPtr<const Port> portProvider;
  };
public:
  PortSet(const Port* sourcePort = 0)
  {
    if (!sourcePort)
      return;
    PathPort pathPort;
    pathPort.portProvider = sourcePort;

    SharedPtr<const Node> node = sourcePort->getNode();
    if (node)
      // FIXME
      pathPort.modelPath = node->getNodePathList().front();

    mPortList.push_back(pathPort);
  }

  const Port* routeTo(const NodePath& path)
  {
    // could happen if the initialzer failed
    if (mPortList.empty())
      return 0;

    // ok, shortcut for old style connections
    if (path.empty())
      return mPortList.front().portProvider;

    const NodePath& originatingPath = mPortList.front().modelPath;
    // fast return if the models are not connected to the same root system
    if (path.front() != originatingPath.front())
      return 0;

    // first check, if we already have a route
    const Port* portProvider = findProvider(path);
    if (portProvider)
      return portProvider;

    // Compute the iterators for seperating the common part of the model path
    // from the different part
    NodePath::const_iterator mi1 = path.begin();
    NodePath::const_iterator mi2 = originatingPath.begin();
    while (mi1 != path.end() && mi2 != originatingPath.end()) {
      if (*mi1 != *mi2)
        break;
      ++mi1;
      ++mi2;
    }

    if (mi1 != path.end()) {
      // that is: we must first go up that path and search again
      NodePath pathUp = path;
      pathUp.pop_back();
      portProvider = routeTo(pathUp);
      if (!portProvider)
        return 0;
      
      GroupInput* groupInput = new GroupInput(portProvider->getName());
      Group* group = const_cast<Group*>(dynamic_cast<const Group*>(path.back().get()));
      group->addChild(groupInput);

      PathPort pathPort;
      pathPort.modelPath = groupInput->getNodePathList().front();
      pathPort.portProvider = group->getPort(groupInput->getExternalPortIndex());
      mPortList.push_back(pathPort);

      if (!group->connect(portProvider, groupInput->getPort("input")))
        return 0;
      
      return pathPort.portProvider.get();

    } else if (mi2 != originatingPath.end()) {
      // that is: we need to step deeper towards the origin of that port
      NodePath pathDown = path;
      pathDown.push_back(*mi2);
      portProvider = routeTo(pathDown);
      if (!portProvider)
        return 0;

      GroupOutput* groupOutput = new GroupOutput(portProvider->getName());
      Group* group = const_cast<Group*>(dynamic_cast<const Group*>(pathDown.back().get()));
      group->addChild(groupOutput);

      PathPort pathPort;
      pathPort.modelPath = groupOutput->getNodePathList().front();
      pathPort.portProvider = group->getPort(groupOutput->getExternalPortIndex());
      mPortList.push_back(pathPort);

      if (!group->connect(portProvider, groupOutput->getPort("input")))
        return 0;
      
      return pathPort.portProvider.get();

    } else {
      // should not happen, in this case the find provider must have been
      // successful,
      return 0;
    }
  }

  const Port* findProvider(const NodePath& path)
  {
    PortList::iterator i = mPortList.begin();
    while (i != mPortList.end()) {
      if (i->modelPath == path)
        return i->portProvider;
      ++i;
    }

    return 0;
  }

private:
  typedef std::list<PathPort> PortList;
  PortList mPortList;
};












class JSBSimProperty {
public:
  typedef std::pair<SharedPtr<const Port>, SharedPtr<Group> > PortGroupPair;

  void setProvider(const Port* port)
  {
    if (!port)
      return;
    SharedPtr<const Node> node = port->getNode();
    OpenFDMAssert(node);
    NodePathList nodePathList = node->getNodePathList();
    OpenFDMAssert(nodePathList.size() == 1);
    Group* group = dynamic_cast<Group*>(const_cast<Node*>(nodePathList.front().back().get()));
    setProvider(port, group);
  }
  void setProvider(const Port* port, Group* group)
  { setProvider(PortGroupPair(port, group)); }
  void setProvider(const PortGroupPair& portPathPair)
  { mProvider = portPathPair; }
  const SharedPtr<const Port>& getProviderPort() const
  { return mProvider.first; }
  const SharedPtr<Group>& getProviderGroup() const
  { return mProvider.second; }

  bool hasProviderPort() const
  { return mProvider.first.valid(); }

  void addConsumer(const Port* port)
  {
    if (!port)
      return;
    SharedPtr<const Node> node = port->getNode();
    OpenFDMAssert(node);
    NodePathList nodePathList = node->getNodePathList();
    OpenFDMAssert(nodePathList.size() == 1);
    Group* group = dynamic_cast<Group*>(const_cast<Node*>(nodePathList.front().back().get()));
    addConsumer(port, group);
  }
  void addConsumer(const Port* port, Group* group)
  { addConsumer(PortGroupPair(port, group)); }
  void addConsumer(const PortGroupPair& portPathPair)
  { mConsumers.push_back(portPathPair); }

  /// Connect all the loose ends stored here
  bool connect()
  {
    if (!hasProviderPort())
      return false;

    PortSet portSet(mProvider.first);
    for (unsigned i = 0; i < mConsumers.size(); ++i) {
      SharedPtr<const Node> node = mConsumers[i].first->getNode();
      OpenFDMAssert(node);
      NodePathList nodePathList = node->getNodePathList();
      const Port* p = portSet.routeTo(nodePathList.front());
      if (!mConsumers[i].second->connect(p, mConsumers[i].first))
        return false;
    }

    return true;
  }

  static std::string simplify(std::string path)
  {
    std::string::size_type idx;
    while ((idx = path.find("[0]")) != std::string::npos) {
      path.erase(idx, 3);
    }
    while ((idx = path.find("//")) != std::string::npos) {
      path.erase(idx, 1);
    }
    return path;
  }
  static std::string propertyPath(const std::string& path)
  {
    std::string simplePath = simplify(path);
    std::string::size_type idx = simplePath.rfind('/');
    if (idx == std::string::npos)
      return std::string();
    simplePath = simplePath.substr(0, idx);
    while (!simplePath.empty() && simplePath[simplePath.size() - 1] == '/')
      simplePath = simplePath.substr(0, simplePath.size() - 1);
    return simplePath.substr(0, idx);
  }
  static std::string propertyName(const std::string& path)
  {
    std::string simplePath = simplify(path);
    std::string::size_type idx = simplePath.rfind('/');
    if (idx == std::string::npos)
      return simplePath;
    return simplePath.substr(idx+1);
  }
  static bool startsWith(const std::string& path, const char* start)
  {
    return path.find(start) == 0;
  }
  static bool endsWith(const std::string& path, const char* start)
  {
    return path.size() - path.rfind(start) == strlen(start);
  }

private:
  /// The port that delivers the value
  PortGroupPair mProvider;
  /// The ports that need that value as input
  std::vector<PortGroupPair> mConsumers;
};

class JSBSimPropertyManager {
public:
  typedef std::map<std::string,JSBSimProperty> PropertyMap;

  const PropertyMap& getPropertyMap() const
  { return mPropertyMap; }
  PropertyMap& getPropertyMap()
  { return mPropertyMap; }

  /// Clears the property map. Is used in the readers to reuse a reader
  void clear()
  { mPropertyMap.clear(); }

  /// Set the port providing this property value
  void setProvider(const std::string& name, const Port* port)
  { mPropertyMap[JSBSimProperty::simplify(name)].setProvider(port); }

  /// Add a port that consumes this property value
  void addConsumer(const std::string& name, const Port* port)
  { mPropertyMap[JSBSimProperty::simplify(name)].addConsumer(port); }

  /// Returns true if this property is already registered
  bool exists(const std::string& propertyName) const
  {
    PropertyMap::const_iterator i = mPropertyMap.find(JSBSimProperty::simplify(propertyName));
    if (i == mPropertyMap.end())
      return false;
    return i->second.hasProviderPort();
  }

  bool connect()
  {
    for (PropertyMap::iterator i = mPropertyMap.begin();
         i != mPropertyMap.end(); ++i) {
      if (!i->second.connect()) {
        std::cerr << "Connecting " << i->first << " failed" << std::endl;
        return false;
      }
    }
    return true;
  }

private:
  PropertyMap mPropertyMap;
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
  // Deprecated
  // const Port* lookupJSBExpression2(const std::string& name,
  //                                 const NodePath& path = NodePath(),
  //                                 bool recheckAeroProp = true);

  // FIXME
  const Port* lookupJSBExpression(const std::string& name,
                                  const NodePath& path = NodePath(),
                                  bool recheckAeroProp = true);

  bool connectJSBExpression(const std::string& name, const Port*,
                            bool recheckAeroProp = true);

  std::string canonicalJSBProperty(std::string name);

  void registerExpression(const std::string& name, const Port* expr);
  void registerJSBExpression(const std::string& name, const Port* expr);

  bool provideSubstitutes();
  bool provideSubstitute(const std::string&, const JSBSimProperty& property);

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

  static SharedPtr<Group> getGroup(const Port* in);
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
