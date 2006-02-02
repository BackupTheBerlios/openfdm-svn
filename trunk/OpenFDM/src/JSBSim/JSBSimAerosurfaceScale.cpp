/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Table.h>

#include "JSBSimAerosurfaceScale.h"




#include <OpenFDM/Gain.h>

namespace OpenFDM {

JSBSimAerosurfaceScale::JSBSimAerosurfaceScale(const std::string& name) :
  JSBSimFCSComponent(name, true)
{
  // Such a component is a simple table lookup
  //
  // -|InputSaturation|-|TablePreLookup|-|Table1D|-
  //

  Saturation* inputSaturation = new Saturation("Input Saturation");
  getModelGroup()->addModel(inputSaturation);
  Matrix tmp(1, 1);
  tmp(1, 1) = -1;
  inputSaturation->setMinSaturation(tmp);
  tmp(1, 1) = 1;
  inputSaturation->setMaxSaturation(tmp);

  mTablePreLookup = new TablePreLookup("Table Lookup");
  TableLookup tl;
  tl.setAtIndex(1, -1);
  tl.setAtIndex(2, 0);
  tl.setAtIndex(3, 1);
  mTablePreLookup->setTableLookup(tl);
  mTablePreLookup->getInputPort(0)->connect(inputSaturation->getOutputPort(0));
  getModelGroup()->addModel(mTablePreLookup);

  mTable = new Table1D("Table");
  TableData<1>::SizeVector sv;
  sv(1) = 3;
  TableData<1> tableData(sv);
  TableData<1>::Index iv;
  iv(1) = 1;
  tableData(iv) = -1;
  iv(1) = 2;
  tableData(iv) = 0;
  iv(1) = 3;
  tableData(iv) = 1;
  mTable->setTableData(tableData);
  mTable->getInputPort(0)->connect(mTablePreLookup->getOutputPort(0));
  getModelGroup()->addModel(mTable);

  // Now connect the input and the output to this groups in and outputs
  getModelGroup()->setNumInputPorts(1);
  getModelGroup()->getInputPort(0)->setName("Input");
  inputSaturation->getInputPort(0)->connect(getModelGroup()->getInputPort(0));

  getOutputPort()->connect(mTable->getOutputPort(0));
  getOutputNormPort()->connect(inputSaturation->getOutputPort(0));
}

JSBSimAerosurfaceScale::~JSBSimAerosurfaceScale(void)
{
}

void
JSBSimAerosurfaceScale::setMinDomain(real_type minDomain)
{
  TableLookup tl = mTablePreLookup->getTableLookup();
  tl.setAtIndex(1, minDomain);
  mTablePreLookup->setTableLookup(tl);
}

void
JSBSimAerosurfaceScale::setMaxDomain(real_type maxDomain)
{
  TableLookup tl = mTablePreLookup->getTableLookup();
  tl.setAtIndex(tl.size(), maxDomain);
  mTablePreLookup->setTableLookup(tl);
}

void
JSBSimAerosurfaceScale::setCentered(bool centered)
{
  TableLookup tlOld = mTablePreLookup->getTableLookup();
  TableLookup tl;
  if (centered) {
    tl.setAtIndex(1, tlOld.getAtIndex(1));
    tl.setAtIndex(2, 0);
    tl.setAtIndex(3, tlOld.getAtIndex(tlOld.size()));
  } else {
    tl.setAtIndex(1, tlOld.getAtIndex(1));
    tl.setAtIndex(2, tlOld.getAtIndex(tlOld.size()));
  }
  mTablePreLookup->setTableLookup(tl);
}

void
JSBSimAerosurfaceScale::setMinValue(real_type minValue)
{
  TableData<1> tableData = mTable->getTableData();
  TableData<1>::Index iv;
  iv(1) = 1;
  tableData(iv) = minValue;
  mTable->setTableData(tableData);
}

void
JSBSimAerosurfaceScale::setMaxValue(real_type maxValue)
{
  TableData<1> tableData = mTable->getTableData();
  TableData<1>::Index iv;
  iv(1) = 3;
  tableData(iv) = maxValue;
  mTable->setTableData(tableData);
}

} //namespace OpenFDM
