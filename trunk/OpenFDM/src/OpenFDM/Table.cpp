/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich
 *
 */

#include <map>
#include "Matrix.h"
#include "Vector.h"
#include "Table.h"

#include <iostream>

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(TablePreLookup, Model)
/// FIXME
//   DEF_OPENFDM_PROPERTY(TablePreLookup, LookupVector, Serialized)
  DEF_OPENFDM_PROPERTY(TablePreLookup, TableLookup, Serialized)
  END_OPENFDM_OBJECT_DEF

TablePreLookup::TablePreLookup(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &TablePreLookup::getOutput);
}

TablePreLookup::~TablePreLookup(void)
{
}

bool
TablePreLookup::init(void)
{
  mInputPortHandle = getInputPort(0)->toRealPortHandle();
  if (!mInputPortHandle.isConnected()) {
    Log(Model,Error) << "Input port to TablePreLookup Model \""
                     << getName() << "\" is not connected" << endl;
    return false;
  }
  return true;
}

void
TablePreLookup::output(const TaskInfo&)
{
  OpenFDMAssert(mInputPortHandle.isConnected());
  mOutput = mTableLookup.lookup(mInputPortHandle.getRealValue());
  Log(Model,Debug3) << "Output of TablePreLookup \"" << getName() << "\" "
                    << mOutput << endl;
}

const real_type&
TablePreLookup::getOutput(void) const
{
  return mOutput;
}

BEGIN_OPENFDM_OBJECT_DEF(Table1D, Model)
  DEF_OPENFDM_PROPERTY(Table1D, TableData, Serialized)
  END_OPENFDM_OBJECT_DEF

Table1D::Table1D(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Table1D::getOutput);
}

Table1D::~Table1D(void)
{
}
  
bool
Table1D::init(void)
{
  mInputPortHandle = getInputPort(0)->toRealPortHandle();
  if (!mInputPortHandle.isConnected()) {
    Log(Model,Error) << "Input port to Table1D Model \""
                     << getName() << "\" is not connected" << endl;
    return false;
  }
  return true;
}

void
Table1D::output(const TaskInfo&)
{
  OpenFDMAssert(mInputPortHandle.isConnected());
  TableData<1>::InterpVector interpVec;
  interpVec(1) = mInputPortHandle.getRealValue();
  mOutput = mTableData.interpolate(interpVec);
  Log(Model,Debug3) << "Output of Table1D \"" << getName() << "\" "
                    << mOutput << endl;
}

const real_type&
Table1D::getOutput(void) const
{
  return mOutput;
}

BEGIN_OPENFDM_OBJECT_DEF(Table2D, Model)
  DEF_OPENFDM_PROPERTY(Table2D, TableData, Serialized)
  END_OPENFDM_OBJECT_DEF

Table2D::Table2D(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(2);
  setInputPortName(0, "input 0");
  setInputPortName(1, "input 1");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Table2D::getOutput);
}

Table2D::~Table2D(void)
{
}

bool
Table2D::init(void)
{
  for (unsigned idx = 0; idx < 2; ++idx) {
    mInputPortHandle[idx] = getInputPort(idx)->toRealPortHandle();
    if (!mInputPortHandle[idx].isConnected()) {
      Log(Model,Error) << "Input port to Table2D Model \""
                       << getName() << "\" is not connected" << endl;
      return false;
    }
  }
  return true;
}

void
Table2D::output(const TaskInfo&)
{
  OpenFDMAssert(mInputPortHandle[0].isConnected());
  OpenFDMAssert(mInputPortHandle[1].isConnected());
  TableData<2>::InterpVector interpVec;
  interpVec(1) = mInputPortHandle[0].getRealValue();
  interpVec(2) = mInputPortHandle[1].getRealValue();
  mOutput = mTableData.interpolate(interpVec);
  Log(Model, Debug3) << "Output of Table2D \"" << getName() << "\" "
                     << mOutput << endl;
}

const real_type&
Table2D::getOutput(void) const
{
  return mOutput;
}

BEGIN_OPENFDM_OBJECT_DEF(Table3D, Model)
  DEF_OPENFDM_PROPERTY(Table3D, TableData, Serialized)
  END_OPENFDM_OBJECT_DEF

Table3D::Table3D(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  
  setNumInputPorts(3);
  setInputPortName(0, "input 0");
  setInputPortName(1, "input 1");
  setInputPortName(2, "input 2");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Table3D::getOutput);
}

Table3D::~Table3D(void)
{
}
  
bool
Table3D::init(void)
{
  for (unsigned idx = 0; idx < 3; ++idx) {
    mInputPortHandle[idx] = getInputPort(idx)->toRealPortHandle();
    if (!mInputPortHandle[idx].isConnected()) {
      Log(Model,Error) << "Input port to Table3D Model \""
                       << getName() << "\" is not connected" << endl;
      return false;
    }
  }
  return true;
}

void
Table3D::output(const TaskInfo&)
{
  OpenFDMAssert(mInputPortHandle[0].isConnected());
  OpenFDMAssert(mInputPortHandle[1].isConnected());
  OpenFDMAssert(mInputPortHandle[2].isConnected());
  TableData<3>::InterpVector interpVec;
  interpVec(1) = mInputPortHandle[0].getRealValue();
  interpVec(2) = mInputPortHandle[1].getRealValue();
  interpVec(3) = mInputPortHandle[2].getRealValue();
  mOutput = mTableData.interpolate(interpVec);
  Log(Model, Debug3) << "Output of Table3D \"" << getName() << "\" "
                     << mOutput << endl;
}

const real_type&
Table3D::getOutput(void) const
{
  return mOutput;
}

} // namespace OpenFDM
