/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimPropertyManager_H
#define OpenFDM_JSBSimPropertyManager_H

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

#include "JSBSimAerodynamic.h"
#include "XMLReader.h"

namespace OpenFDM {

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
  bool connect();

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

private:
  PropertyMap mPropertyMap;
};

} // namespace OpenFDM

#endif
