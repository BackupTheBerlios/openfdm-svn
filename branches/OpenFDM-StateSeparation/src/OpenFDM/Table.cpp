/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich
 *
 */

#include "Table.h"

#include <map>
#include <iostream>
#include "Matrix.h"
#include "Vector.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Table1D, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Table1D, TableData, Serialized)
  END_OPENFDM_OBJECT_DEF

Table1D::Table1D(const std::string& name) :
  SimpleDirectModel(name)
{
  setNumInputPorts(1);
}

Table1D::~Table1D(void)
{
}
  
void
Table1D::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(0); ++j) {
    for (unsigned k = 0; k < sz(1); ++k) {
      TableData<1>::InterpVector input;
      input(0) = context.getInputValue(0)(j, k);
      context.getOutputValue()(j, k) = mTableData.interpolate(input);
    }
  }
}

BEGIN_OPENFDM_OBJECT_DEF(Table2D, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Table2D, TableData, Serialized)
  END_OPENFDM_OBJECT_DEF

Table2D::Table2D(const std::string& name) :
  SimpleDirectModel(name)
{
  setNumInputPorts(2);
}

Table2D::~Table2D(void)
{
}

void
Table2D::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(0); ++j) {
    for (unsigned k = 0; k < sz(1); ++k) {
      TableData<2>::InterpVector input;
      input(0) = context.getInputValue(0)(j, k);
      input(1) = context.getInputValue(1)(j, k);
      context.getOutputValue()(j, k) = mTableData.interpolate(input);
    }
  }
}

BEGIN_OPENFDM_OBJECT_DEF(Table3D, SimpleDirectModel)
  DEF_OPENFDM_PROPERTY(Table3D, TableData, Serialized)
  END_OPENFDM_OBJECT_DEF

Table3D::Table3D(const std::string& name) :
  SimpleDirectModel(name)
{
  setNumInputPorts(3);
}

Table3D::~Table3D(void)
{
}
  
void
Table3D::output(Context& context) const
{
  Size sz = size(context.getInputValue(0));
  for (unsigned j = 0; j < sz(0); ++j) {
    for (unsigned k = 0; k < sz(1); ++k) {
      TableData<3>::InterpVector input;
      input(0) = context.getInputValue(0)(j, k);
      input(1) = context.getInputValue(1)(j, k);
      input(2) = context.getInputValue(2)(j, k);
      context.getOutputValue()(j, k) = mTableData.interpolate(input);
    }
  }
}

} // namespace OpenFDM
