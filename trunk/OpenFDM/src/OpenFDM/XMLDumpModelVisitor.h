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
#include "MultiBodySystem.h"

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
    indent() << "<" << model.getTypeName() << ">\n";
    ++mIndent;
    dumpProperties(model);
    --mIndent;
    indent() << "</" << model.getTypeName() << ">\n";
  }
  virtual void apply(ModelGroup& modelGroup)
  {
    indent() << "<ModelGroup>\n";
    ++mIndent;
    dumpProperties(modelGroup);
    traverse(modelGroup);
    --mIndent;
    indent() << "</ModelGroup>\n";
  }
  virtual void apply(MultiBodySystem& multiBodySystem)
  {
    indent() << "<MultiBodySystem>\n";
    ++mIndent;
    dumpProperties(multiBodySystem);
    traverse(multiBodySystem);
    --mIndent;
    indent() << "</MultiBodySystem>\n";
  }
  virtual void apply(System& system)
  {
    indent() << "<?xml version=\"1.0\"?>\n";
    indent() << "<OpenFDM>\n";
    ++mIndent;
    indent() << "<System>\n";
    ++mIndent;
    dumpProperties(system);
    traverse(system);
    --mIndent;
    indent() << "</System>\n";
    --mIndent;
    indent() << "</OpenFDM>\n";
  }
private:
  void dumpProperties(const Object& object)
  {
    std::list<std::string> propNames = object.listProperties();
    std::list<std::string>::iterator it = propNames.begin();
    while (it != propNames.end()) {
      if (object.isStoredProperty(*it)) {
        Variant value = object.getPropertyValue(*it);
        
        if (value.isReal()) {
          std::stringstream valStr;
          valStr.precision(mPrecision);
          valStr  << value.toReal();
          indent() << "<property name=\"" << *it << "\" type=\"real\">"
                   << valStr.str()
                   << "</property>\n";
        } else if (value.isInteger()) {
          indent() << "<property name=\"" << *it << "\" type=\"integer\">"
                   << value.toInteger()
                   << "</property>\n";
        } else if (value.isUnsigned()) {
          indent() << "<property name=\"" << *it << "\" type=\"unsigned\">"
                   << value.toUnsigned()
                   << "</property>\n";
        } else if (value.isMatrix()) {
          Matrix mVal = value.toMatrix();
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
          indent() << "<property name=\"" << *it << "\" type=\"matrix\">"
                   << valStr.str()
                   << "</property>\n";
        } else if (value.isString()) {
          indent() << "<property name=\"" << *it << "\" type=\"string\">"
                   << escape(value.toString())
                   << "</property>\n";
        } else {
          indent() << "<property name=\"" << *it << "\">"
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