/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XMLDumpModelVisitor_H
#define OpenFDM_XMLDumpModelVisitor_H

#include <iosfwd>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "NodeVisitor.h"
#include "Node.h"
#include "Group.h"
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

  virtual void apply(Node& model)
  {
    indent() << "<model type=\"" << model.getTypeName() << "\">\n";
    ++mIndent;
    dumpProperties(model);
    --mIndent;
    indent() << "</model>\n";
  }
  virtual void apply(Group& modelGroup)
  {
    indent() << "<model type=\"ModelGroup\">\n";
    ++mIndent;
    dumpProperties(modelGroup);
    modelGroup.traverse(*this);
    dumpConnections(modelGroup);
    --mIndent;
    indent() << "</model>\n";
  }
//   virtual void apply(System& system)
//   {
//     indent() << "<?xml version=\"1.0\"?>\n";
//     indent() << "<OpenFDM>\n";
//     ++mIndent;
//     indent() << "<model type=\"System\">\n";
//     ++mIndent;
//     dumpProperties(system);
//     traverse(system);
//     dumpConnections(system);
//     --mIndent;
//     indent() << "</model>\n";
//     --mIndent;
//     indent() << "</OpenFDM>\n";
//   }
private:
  void dumpPort(const Port* port)
  {
    if (!port)
      return;
    SharedPtr<const Node> node = port->getNode();
    if (!node)
      return;
    indent() << "<Port ModelName=\"" << node->getName()
             << "\" PortName=\"" << port->getName()
             << "\"/>\n";
  }
  void dumpConnections(const Group& modelGroup)
  {
    unsigned numConnections = modelGroup.getNumConnects();
    for (unsigned i = 0; i < numConnections; ++i) {
      const Connect* connection = modelGroup.getConnect(i);
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
              for (unsigned i = 1; i < rows(mVal); ++i)
                valStr << mVal(i, 1) << "; ";
              valStr << mVal(rows(mVal), 1);
            } else {
              valStr << '\n';
              for (unsigned i = 1; i < rows(mVal); ++i) {
                // Indent too??
                valStr << std::setw((mIndent+1)*mIndentWidth) << ' ';
                valStr << mVal(cols(mVal), i) << '\n';
                for (unsigned j = 1; j < cols(mVal); ++j)
                  valStr << mVal(i, 1) << ' ';
                valStr << mVal(cols(mVal), i) << '\n';
              }
            }
            indent() << "<property name=\"" << it->getName()
                     << "\" type=\"matrix\">"
                     << valStr.str()
                     << "</property>\n";
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
