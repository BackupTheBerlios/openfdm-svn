/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <OpenFDM/GroupInput.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Table.h>

#include "JSBSimAerosurfaceScale.h"

#include <OpenFDM/Gain.h>

namespace OpenFDM {

JSBSimAerosurfaceScale::JSBSimAerosurfaceScale(const std::string& name) :
  JSBSimFCSComponent(name),
  mGain(1)
{
  // Such a component is a simple table lookup
  //
  // -|InputSaturation|-|TablePreLookup|-|Table1D|-
  //

  mInputSaturation = new Saturation("Input Saturation");
  getModelGroup()->addModel(mInputSaturation);
  Matrix tmp(1, 1);
  tmp(1, 1) = -1;
  mInputSaturation->setMinSaturation(tmp);
  tmp(1, 1) = 1;
  mInputSaturation->setMaxSaturation(tmp);

  mTablePreLookup = new TablePreLookup("Table Lookup");
  TableLookup tl;
  tl.setAtIndex(1, -1);
  tl.setAtIndex(2, 0);
  tl.setAtIndex(3, 1);
  mTablePreLookup->setTableLookup(tl);
  getModelGroup()->addModel(mTablePreLookup);
  Connection::connect(mTablePreLookup->getInputPort(0),
                      mInputSaturation->getOutputPort(0));

  mTable = new Table1D("Table");
  TableData<1>::SizeVector sv;
  sv(1) = 3;
  TableData<1> tableData(sv);
  TableData<1>::Index iv;
  iv(1) = 1;
  tableData(iv) = -mGain;
  iv(1) = 2;
  tableData(iv) = 0;
  iv(1) = 3;
  tableData(iv) = mGain;
  mTable->setTableData(tableData);
  getModelGroup()->addModel(mTable);
  Connection::connect(mTable->getInputPort(0),
                      mTablePreLookup->getOutputPort(0));

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
  tmp(1, 1) = minDomain;
  mInputSaturation->setMinSaturation(tmp);
  TableLookup tl = mTablePreLookup->getTableLookup();
  tl.setAtIndex(1, minDomain);
  mTablePreLookup->setTableLookup(tl);
}

void
JSBSimAerosurfaceScale::setMaxDomain(real_type maxDomain)
{
  Matrix tmp(1, 1);
  tmp(1, 1) = maxDomain;
  mInputSaturation->setMaxSaturation(tmp);
  TableLookup tl = mTablePreLookup->getTableLookup();
  tl.setAtIndex(tl.size(), maxDomain);
  mTablePreLookup->setTableLookup(tl);
}

void
JSBSimAerosurfaceScale::setCentered(bool centered)
{
  TableLookup tlOld = mTablePreLookup->getTableLookup();
  TableLookup tl;

  TableData<1> tableDataOld = mTable->getTableData();
  TableData<1>::SizeVector sz;
  TableData<1> tableData;
  TableData<1>::Index iv;

  if (centered) {
    tl.setAtIndex(1, tlOld.getAtIndex(1));
    tl.setAtIndex(2, 0);
    tl.setAtIndex(3, tlOld.getAtIndex(tlOld.size()));

    sz(1) = 3;
    tableData = TableData<1>(sz);
    iv(1) = 1;
    tableData(iv) = tableDataOld(iv);
    iv(1) = 2;
    tableData(iv) = 0;
    iv = tableDataOld.size();
    real_type oldVal = tableDataOld(iv);
    iv(1) = 3;
    tableData(iv) = oldVal;
  } else {
    tl.setAtIndex(1, tlOld.getAtIndex(1));
    tl.setAtIndex(2, tlOld.getAtIndex(tlOld.size()));

    sz(1) = 2;
    tableData = TableData<1>(sz);
    iv(1) = 1;
    tableData(iv) = tableDataOld(iv);
    iv = tableDataOld.size();
    real_type oldVal = tableDataOld(iv);
    iv(1) = 2;
    tableData(iv) = oldVal;
  }

  mTablePreLookup->setTableLookup(tl);
  mTable->setTableData(tableData);
}

void
JSBSimAerosurfaceScale::setMinValue(real_type minValue)
{
  TableData<1> tableData = mTable->getTableData();
  TableData<1>::Index iv;
  iv(1) = 1;
  tableData(iv) = mGain*minValue;
  mTable->setTableData(tableData);
}

void
JSBSimAerosurfaceScale::setMaxValue(real_type maxValue)
{
  TableLookup tl = mTablePreLookup->getTableLookup();
  TableData<1> tableData = mTable->getTableData();
  TableData<1>::Index iv;
  iv(1) = tl.size();
  tableData(iv) = mGain*maxValue;
  mTable->setTableData(tableData);
}

void
JSBSimAerosurfaceScale::setGain(real_type gain)
{
  TableLookup tl = mTablePreLookup->getTableLookup();
  TableData<1> tableData = mTable->getTableData();
  TableData<1>::Index iv;
  iv(1) = 1;
  tableData(iv) *= gain/mGain;
  iv(1) = tl.size();
  tableData(iv) *= gain/mGain;
  mTable->setTableData(tableData);
  mGain = gain;
}

} //namespace OpenFDM
