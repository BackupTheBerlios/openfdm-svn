/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <OpenFDM/GroupInput.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Table.h>

#include "JSBSimScheduledGain.h"

namespace OpenFDM {

JSBSimScheduledGain::JSBSimScheduledGain(const std::string& name) :
  JSBSimFCSComponent(name)
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
  Connection::connect(product->getInputPort(0),
                      mTable->getOutputPort(0));
  Connection::connect(mTable->getInputPort(0),
                      mTablePreLookup->getOutputPort(0));

  // Now connect the input and the output to this groups in and outputs
  GroupInput* groupInput = new GroupInput("Input");
  getModelGroup()->addModel(groupInput);
  Connection::connect(product->getInputPort(1),
                      groupInput->getOutputPort(0));
 
  groupInput = new GroupInput("Schedule Input");
  getModelGroup()->addModel(groupInput);
  Connection::connect(mTablePreLookup->getInputPort(0),
                      groupInput->getOutputPort(0));

  // That single output port is this one
  Connection::connect(getInternalOutputPort(),
                      product->getOutputPort(0));
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
