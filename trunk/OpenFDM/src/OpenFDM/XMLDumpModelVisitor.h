/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XMLDumpModelVisitor_H
#define OpenFDM_XMLDumpModelVisitor_H

#include <iosfwd>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "ModelVisitor.h"
#include "Model.h"
#include "ModelGroup.h"

namespace OpenFDM {

class XMLDumpModelVisitor : public ModelVisitor {
public:
  XMLDumpModelVisitor(std::ostream& os) :
    mOs(os),
    mIndent(0u),
    mIndentWidth(2),
    mPrecision(15)
  { }
  virtual ~XMLDumpModelVisitor(void)
  { }

  virtual void apply(Model& model)
  {
    indent() << "<model type=\"" << model.getTypeName() << "\">\n";
    ++mIndent;
    dumpProperties(model);
    --mIndent;
    indent() << "</model>\n";
  }
  virtual void apply(ModelGroup& modelGroup)
  {
    indent() << "<model type=\"ModelGroup\">\n";
    ++mIndent;
    dumpProperties(modelGroup);
    traverse(modelGroup);
    dumpConnections(modelGroup);
    --mIndent;
    indent() << "</model>\n";
  }
  virtual void apply(System& system)
  {
    indent() << "<?xml version=\"1.0\"?>\n";
    indent() << "<OpenFDM>\n";
    ++mIndent;
    indent() << "<model type=\"System\">\n";
    ++mIndent;
    dumpProperties(system);
    traverse(system);
    dumpConnections(system);
    --mIndent;
    indent() << "</model>\n";
    --mIndent;
    indent() << "</OpenFDM>\n";
  }
private:
  void dumpConnections(const ModelGroup& modelGroup)
  {
    unsigned numConnections = modelGroup.getNumConnections();
    for (unsigned i = 0; i < numConnections; ++i) {
      const Connection* connection = modelGroup.getConnection(i);
      if (connection->getName().empty())
        indent() << "<connect>\n";
      else
        indent() << "<connect name=\"" << connection->getName() << "\">\n";
      ++mIndent;
      dumpProperties(*connection);
      const PortProvider* portProvider = connection->getPortProvider();
      if (portProvider) {
        SharedPtr<Model> model = portProvider->getModel().lock();
        if (model) {
          indent() << "<PortProvider ModelName=\"" << model->getName()
                   << "\" PortName=\"" << portProvider->getName()
                   << "\"/>\n";
        }
      }
      const PortAcceptor* portAcceptor = connection->getPortAcceptor();
      if (portAcceptor) {
        SharedPtr<Model> model = portAcceptor->getModel().lock();
        if (model) {
          indent() << "<PortAcceptor ModelName=\"" << model->getName()
                   << "\" PortName=\"" << portAcceptor->getName()
                   << "\"/>\n";
        }
      }
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
