/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "JSBSimAerosurfaceScale.h"

#include <OpenFDM/GroupInput.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Table.h>
#include <OpenFDM/Gain.h>

namespace OpenFDM {

JSBSimAerosurfaceScale::JSBSimAerosurfaceScale(const std::string& name) :
  JSBSimFCSComponent(name),
  mGain(1)
{
  // Such a component is a simple table lookup
  //
  // -|InputSaturation|-|BreakPointLookup|-|Table1D|-
  //

  mInputSaturation = new Saturation("Input Saturation");
  getModelGroup()->addModel(mInputSaturation);
  Matrix tmp(1, 1);
  tmp(0, 0) = -1;
  mInputSaturation->setMinSaturation(tmp);
  tmp(0, 0) = 1;
  mInputSaturation->setMaxSaturation(tmp);

  mBreakPointLookup = new BreakPointLookup("Table Lookup");
  BreakPointVector tl;
  tl.setAtIndex(0, -1);
  tl.setAtIndex(1, 0);
  tl.setAtIndex(2, 1);
  mBreakPointLookup->setBreakPointVector(tl);
  getModelGroup()->addModel(mBreakPointLookup);
  Connection::connect(mBreakPointLookup->getInputPort(0),
                      mInputSaturation->getOutputPort(0));

  mTable = new Table1D("Table");
  TableData<1>::SizeVector sv;
  sv(0) = 3;
  TableData<1> tableData(sv);
  TableData<1>::Index iv;
  iv(0) = 0;
  tableData(iv) = -mGain;
  iv(0) = 1;
  tableData(iv) = 0;
  iv(0) = 2;
  tableData(iv) = mGain;
  mTable->setTableData(tableData);
  getModelGroup()->addModel(mTable);
  Connection::connect(mTable->getInputPort(0),
                      mBreakPointLookup->getOutputPort(0));

  // Now connect the input and the output to this groups in and outputs
  GroupInput* groupInput = new GroupInput("Input");
  getModelGroup()->addModel(groupInput);
  Connection::connect(mInputSaturation->getInputPort(0),
                      groupInput->getOutputPort(0));
 
  // That single output port is this one
  Connection::connect(getInternalOutputPort(),
                      mTable->getOutputPort(0));
  // FIXME, is no longer normalized ...
  Connection::connect(getInternalOutputNormPort(),
                      mInputSaturation->getOutputPort(0));
}

JSBSimAerosurfaceScale::~JSBSimAerosurfaceScale(void)
{
}

void
JSBSimAerosurfaceScale::setMinDomain(real_type minDomain)
{
  Matrix tmp(1, 1);
  tmp(0, 0) = minDomain;
  mInputSaturation->setMinSaturation(tmp);
  BreakPointVector tl = mBreakPointLookup->getBreakPointVector();
  tl.setAtIndex(0, minDomain);
  mBreakPointLookup->setBreakPointVector(tl);
}

void
JSBSimAerosurfaceScale::setMaxDomain(real_type maxDomain)
{
  Matrix tmp(1, 1);
  tmp(0, 0) = maxDomain;
  mInputSaturation->setMaxSaturation(tmp);
  BreakPointVector tl = mBreakPointLookup->getBreakPointVector();
  tl.setAtIndex(tl.size()-1, maxDomain);
  mBreakPointLookup->setBreakPointVector(tl);
}

void
JSBSimAerosurfaceScale::setCentered(bool centered)
{
  BreakPointVector tlOld = mBreakPointLookup->getBreakPointVector();
  BreakPointVector tl;

  TableData<1> tableDataOld = mTable->getTableData();
  TableData<1>::SizeVector sz;
  TableData<1> tableData;
  TableData<1>::Index iv;

  if (centered) {
    tl.setAtIndex(0, tlOld.getAtIndex(0));
    tl.setAtIndex(1, 0);
    tl.setAtIndex(2, tlOld.getAtIndex(tlOld.size()-1));

    sz(0) = 3;
    tableData = TableData<1>(sz);
    iv(0) = 0;
    tableData(iv) = tableDataOld(iv);
    iv(0) = 1;
    tableData(iv) = 0;
    iv(0) = tableDataOld.size(0)-1;
    real_type oldVal = tableDataOld(iv);
    iv(0) = 2;
    tableData(iv) = oldVal;
  } else {
    tl.setAtIndex(0, tlOld.getAtIndex(0));
    tl.setAtIndex(1, tlOld.getAtIndex(tlOld.size()-1));

    sz(0) = 2;
    tableData = TableData<1>(sz);
    iv(0) = 0;
    tableData(iv) = tableDataOld(iv);
    iv(0) = tableDataOld.size(0)-1;
    real_type oldVal = tableDataOld(iv);
    iv(0) = 1;
    tableData(iv) = oldVal;
  }

  mBreakPointLookup->setBreakPointVector(tl);
  mTable->setTableData(tableData);
}

void
JSBSimAerosurfaceScale::setMinValue(real_type minValue)
{
  TableData<1> tableData = mTable->getTableData();
  TableData<1>::Index iv;
  iv(0) = 0;
  tableData(iv) = mGain*minValue;
  mTable->setTableData(tableData);
}

void
JSBSimAerosurfaceScale::setMaxValue(real_type maxValue)
{
  BreakPointVector tl = mBreakPointLookup->getBreakPointVector();
  TableData<1> tableData = mTable->getTableData();
  TableData<1>::Index iv;
  iv(0) = tl.size()-1;
  tableData(iv) = mGain*maxValue;
  mTable->setTableData(tableData);
}

void
JSBSimAerosurfaceScale::setGain(real_type gain)
{
  BreakPointVector tl = mBreakPointLookup->getBreakPointVector();
  TableData<1> tableData = mTable->getTableData();
  TableData<1>::Index iv;
  iv(0) = 0;
  tableData(iv) *= gain/mGain;
  iv(0) = tl.size()-1;
  tableData(iv) *= gain/mGain;
  mTable->setTableData(tableData);
  mGain = gain;
}

} //namespace OpenFDM
