/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Table.h>

#include "JSBSimScheduledGain.h"

namespace OpenFDM {

JSBSimScheduledGain::JSBSimScheduledGain(const std::string& name) :
  JSBSimFCSComponent(name, false)
{
  // Such a component is a simple product and a table lookup
  //
  // -------------------------------|
  // --|TablePreLookup|-|Table|-|Product|-
  //

  Product* product = new Product("Product");
  getModelGroup()->addModel(product);

  mTablePreLookup = new TablePreLookup("Table Lookup");
  getModelGroup()->addModel(mTablePreLookup);

  mTable = new Table1D("Table");
  getModelGroup()->addModel(mTable);
  product->getInputPort(0)->connect(mTable->getOutputPort(0));
  mTable->getInputPort(0)->connect(mTablePreLookup->getOutputPort(0));

  // Now connect the input and the output to this groups in and outputs
  getModelGroup()->setNumInputPorts(2);
  getModelGroup()->getInputPort(1)->setName("Schedule Input");
  mTablePreLookup->getInputPort(0)->connect(getModelGroup()->getInputPort(1));

  getModelGroup()->getInputPort(0)->setName("Input");
  product->getInputPort(1)->connect(getModelGroup()->getInputPort(0));

  // That single output port is this one
  getOutputPort()->connect(product->getOutputPort(0));
}

JSBSimScheduledGain::~JSBSimScheduledGain(void)
{
}

void
JSBSimScheduledGain::setTableData(const TableData<1>& tableData,
                                  const TableLookup& lookup)
{
  mTable->setTableData(tableData);
  mTablePreLookup->setTableLookup(lookup);
}

} //namespace OpenFDM
