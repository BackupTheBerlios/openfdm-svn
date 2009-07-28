/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
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
#include <OpenFDM/Model.h>
#include <OpenFDM/GroupInput.h>
#include <OpenFDM/GroupOutput.h>
#include <OpenFDM/Connection.h>
#include <OpenFDM/Port.h>

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

class PortSet {
  struct PathPort {
    Node::GroupPath modelPath;
    SharedPtr<Port> portProvider;
  };
public:
  PortSet(Port* sourcePort = 0)
  {
    if (!sourcePort)
      return;
    PathPort pathPort;
    pathPort.portProvider = sourcePort;

    SharedPtr<Node> node = sourcePort->getModel().lock();
    if (node)
      pathPort.modelPath = node->getPath();

    mPortList.push_back(pathPort);
  }

  Port* routeTo(const Node::GroupPath& path)
  {
    // could happen if the initialzer failed
    if (mPortList.empty())
      return 0;

    // ok, shortcut for old style connections
    if (path.empty())
      return mPortList.front().portProvider;

    const Node::GroupPath& originatingPath = mPortList.front().modelPath;
    // fast return if the models are not connected to the same root system
    if (path.front() != originatingPath.front())
      return 0;

    // first check, if we already have a route
    Port* portProvider = findProvider(path);
    if (portProvider)
      return portProvider;

    // Compute the iterators for seperating the common part of the model path
    // from the different part
    Node::GroupPath::const_iterator mi1 = path.begin();
    Node::GroupPath::const_iterator mi2 = originatingPath.begin();
    while (mi1 != path.end() && mi2 != originatingPath.end()) {
      if (*mi1 != *mi2)
        break;
      ++mi1;
      ++mi2;
    }

    if (mi1 != path.end()) {
      // that is: we must first go up that path and search again
      Node::GroupPath pathUp = path;
      pathUp.pop_back();
      portProvider = routeTo(pathUp);
      if (!portProvider)
        return 0;
      
      GroupInput* groupInput = new GroupInput(portProvider->getName());
      path.back()->addChild(groupInput, true);

      PathPort pathPort;
      pathPort.modelPath = groupInput->getPath();
      pathPort.portProvider = groupInput->getOutputPort(0);
      mPortList.push_back(pathPort);

      Connection::connect(portProvider, groupInput->getGroupInput());
      
      return groupInput->getOutputPort(0);

    } else if (mi2 != originatingPath.end()) {
      // that is: we need to step deeper towards the origin of that port
      Node::GroupPath pathDown = path;
      pathDown.push_back(*mi2);
      portProvider = routeTo(pathDown);
      if (!portProvider)
        return 0;

      GroupOutput* groupOutput = new GroupOutput(portProvider->getName());
      pathDown.back()->addChild(groupOutput, true);

      PathPort pathPort;
      pathPort.modelPath = groupOutput->getPath();
      pathPort.portProvider = groupOutput->getGroupOutput();
      mPortList.push_back(pathPort);

      Connection::connect(portProvider, groupOutput->getInputPort(0));
      
      return groupOutput->getGroupOutput();

    } else {
      // should not happen, in this case the find provider must have been
      // successful,
      return 0;
    }
  }

  Port* findProvider(const Node::GroupPath& path)
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

// Implements a SimGear SGProperty compatible 'path' to 'expression'
// mapping.
// It is used to map the value names occuring in JSBSim configuration files
// to OpenFDM::ObsoleteExpressions which will contain the values later.
// This is done by wrapping all map accesses where a key_type argument is given
// with a function which first simplyfys the given key and then performs the
// map operations with the simplyfied key instead of the original one.
// 
/// FIXME!!!!!!!!
class PropertyMap {
public:
  static std::string simplify(std::string path)
  {
    std::string::size_type idx;
    while ((idx = path.find("[0]")) != std::string::npos) {
      path.erase(idx, 3);
    }
    return path;
  }

  /// Clears the property map. Is used in the readers to reuse a reader
  void clear()
  { mMap.clear(); }

  Port* routeTo(const std::string& name, const Node::GroupPath& path)
  {
    std::string simplifiedName = simplify(name);
    if (mMap.count(simplifiedName) <= 0)
      return 0;
    return mMap[simplifiedName].routeTo(path);
  }

  void registerPort(const std::string& name, Port* port)
  { mMap[simplify(name)] = port; }

  /// Returns true if this property is already registered
  bool exists(const std::string& propertyName) const
  { return 0 < mMap.count(simplify(propertyName)); }

private:
  typedef std::map<std::string,PortSet> PortMap;

  PortMap mMap;
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
                                    const Node::GroupPath& path = Node::GroupPath(),
                                    bool recheckAeroProp = true);

  bool connectJSBExpression(const std::string& name, Port*,
                            bool recheckAeroProp = true);

  void registerExpression(const std::string& name, Port* expr);
  void registerJSBExpression(const std::string& name, Port* expr);

  Port* createAndScheduleAeroProp(const std::string& name,
                                          const Node::GroupPath& path);
  Port* createAndScheduleInput(const std::string& name,
                                       const Node::GroupPath& path);

  Port* addInputModel(const std::string& name, const std::string& propName,
                      real_type gain = 1);
  void addOutputModel(Port* out, const std::string& name,
                      const std::string& propName, real_type gain = 1);

  Port* addInverterModel(const std::string& name, Port* in);
  Port* addAbsModel(const std::string& name, Port* in);
  Port* addConstModel(const std::string& name, real_type value,
                              const Node::GroupPath& path = Node::GroupPath());
  Port* addToUnit(const std::string& name, Unit u, Port* in);
  Port* addFromUnit(const std::string& name, Unit u, Port* in);

  static SharedPtr<Group> getGroup(Port* in);
  void addFCSModel(Node* model);

  Port* addMultiBodyConstModel(const std::string& name, real_type value);
  void addMultiBodyModel(Model* model);
  /// </FIXME> document and rethink
  Port* getTablePrelookup(const std::string& name, Port* in, const BreakPointVector& tl);
  /// List for the aircraft search path.
  std::list<std::string> mAircraftPath;
  /// List for the engine search path.
  std::list<std::string> mEnginePath;

  PropertyMap mExpressionTable;
  SharedPtr<AeroForce> mAeroForce;
  std::vector<SharedPtr<BreakPointLookup> > mBreakPointVectors;

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
