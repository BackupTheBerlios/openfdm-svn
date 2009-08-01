/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XMLDumpModelVisitor_H
#define OpenFDM_XMLDumpModelVisitor_H

#include <iosfwd>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "Group.h"
#include "Node.h"
#include "NodeVisitor.h"
#include "System.h"
#include "TypeInfo.h"

namespace OpenFDM {

class XMLDumpModelVisitor : public NodeVisitor {
public:
  XMLDumpModelVisitor(std::ostream& os) :
    mOs(os),
    mIndent(0u),
    mIndentWidth(2),
    mPrecision(15)
  { }
  virtual ~XMLDumpModelVisitor(void)
  { }

  virtual void apply(Node& node)
  {
    indent() << "<node type=\"" << node.getTypeName() << "\">\n";
    ++mIndent;
    dumpProperties(node);
    --mIndent;
    indent() << "</node>\n";
  }
  virtual void apply(Group& group)
  {
    indent() << "<node type=\"NodeGroup\">\n";
    ++mIndent;
    dumpProperties(group);
    group.traverse(*this);
    dumpConnections(group);
    --mIndent;
    indent() << "</node>\n";
  }
  void apply(System& system)
  {
    indent() << "<?xml version=\"1.0\"?>\n";
    indent() << "<OpenFDM>\n";
    ++mIndent;
    indent() << "<System>\n";
    ++mIndent;
    dumpProperties(system);
    system.getNode()->accept(*this);
    --mIndent;
    indent() << "</System>\n";
    --mIndent;
    indent() << "</OpenFDM>\n";
  }
private:
  void dumpPort(const Port* port)
  {
    if (!port)
      return;
    SharedPtr<const Node> node = port->getNode();
    if (!node)
      return;
    indent() << "<port nodeName=\"" << node->getName()
             << "\" portName=\"" << port->getName()
             << "\"/>\n";
  }
  void dumpConnections(const Group& group)
  {
    unsigned numConnections = group.getNumConnects();
    for (unsigned i = 0; i < numConnections; ++i) {
      const Connect* connection = group.getConnect(i);
      indent() << "<connect>\n";
      ++mIndent;
      dumpPort(connection->getPort0());
      dumpPort(connection->getPort1());
      --mIndent;
      indent() << "</connect>\n";
    }
  }
  void dumpProperties(const Object& object)
  {
    std::vector<PropertyInfo> props;
    object.getPropertyInfoList(props);

    std::vector<PropertyInfo>::iterator it = props.begin();
    while (it != props.end()) {
      if (it->isSerialized()) {
        Variant value;
        object.getPropertyValue(it->getName(), value);
        
        if (value.isReal()) {
          std::stringstream valStr;
          valStr.precision(mPrecision);
          valStr  << value.toReal();
          indent() << "<property name=\"" << it->getName()
                   << "\" type=\"real\">"
                   << valStr.str()
                   << "</property>\n";
        } else if (value.isInteger()) {
          indent() << "<property name=\"" << it->getName()
                   << "\" type=\"integer\">"
                   << value.toInteger()
                   << "</property>\n";
        } else if (value.isUnsigned()) {
          indent() << "<property name=\"" << it->getName()
                   << "\" type=\"unsigned\">"
                   << value.toUnsigned()
                   << "</property>\n";
        } else if (value.isMatrix()) {
          Matrix mVal = value.toMatrix();
          if (cols(mVal) != 0) {
            std::stringstream valStr;
            valStr.precision(mPrecision);
            if (cols(mVal) == 1) {
              // A pure vector is written transposed
              for (unsigned i = 0; i < rows(mVal) - 1; ++i)
                valStr << mVal(i, 0) << "; ";
              valStr << mVal(rows(mVal) - 1, 0);

              indent() << "<property name=\"" << it->getName()
                       << "\" type=\"matrix\">"
                       << valStr.str()
                       << "</property>\n";
            } else {
              valStr << '\n';
              for (unsigned i = 0; i < rows(mVal); ++i) {
                // Indent too??
                valStr << std::setw((mIndent+1)*mIndentWidth) << ' ';
                for (unsigned j = 0; j < cols(mVal) - 1; ++j)
                  valStr << mVal(i, j) << ' ';
                valStr << mVal(i, cols(mVal) - 1) << '\n';
              }
              indent() << "<property name=\"" << it->getName()
                       << "\" type=\"matrix\">"
                       << valStr.str();
              indent() << "</property>\n";
            }
          }
        } else if (value.isString()) {
          std::string stringValue = value.toString();
          if (!stringValue.empty())
            indent() << "<property name=\"" << it->getName()
                     << "\" type=\"string\">"
                     << escape(stringValue)
                     << "</property>\n";
        } else {
          indent() << "<property name=\"" << it->getName()
                   << "\">"
                   << escape(value.toString())
                   << "</property>\n";
        }
      }
      ++it;
    }
  }
  std::ostream& indent(void)
  {
    unsigned iw = mIndentWidth*mIndent;
    if (iw)
      mOs << std::setw(iw) << ' ';
    return mOs;
  }

  static std::string escape(std::string s)
  {
    std::string::size_type reppos;
    while ((reppos = s.find(">")) != std::string::npos)
      s.replace(reppos, 1, "&gt;");
    while ((reppos = s.find("<")) != std::string::npos)
      s.replace(reppos, 1, "&lt;");
    return s;
  }

  std::ostream& mOs;
  unsigned mIndent;

  /// Expose them to the user ...
  unsigned mIndentWidth;
  unsigned mPrecision;
};

} // namespace OpenFDM

#endif
