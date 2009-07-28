/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "JSBSimScheduledGain.h"

#include <OpenFDM/GroupInput.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Table.h>

namespace OpenFDM {

JSBSimScheduledGain::JSBSimScheduledGain(const std::string& name) :
  JSBSimFCSComponent(name)
{
  // Such a component is a simple product and a table lookup
  //
  // -------------------------------|
  // --|BreakPointLookup|-|Table|-|Product|-
  //

  Product* product = new Product("Product");
  getGroup()->addChild(product);

  mBreakPointLookup = new BreakPointLookup("Table Lookup");
  getGroup()->addChild(mBreakPointLookup);

  mTable = new Table1D("Table");
  getGroup()->addChild(mTable);
  Connection::connect(product->getInputPort(0),
                      mTable->getOutputPort(0));
  Connection::connect(mTable->getInputPort(0),
                      mBreakPointLookup->getOutputPort(0));

  // Now connect the input and the output to this groups in and outputs
  GroupInput* groupInput = new GroupInput("Input");
  getGroup()->addChild(groupInput);
  Connection::connect(product->getInputPort(1),
                      groupInput->getOutputPort(0));
 
  groupInput = new GroupInput("Schedule Input");
  getGroup()->addChild(groupInput);
  Connection::connect(mBreakPointLookup->getInputPort(0),
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
                                  const BreakPointVector& lookup)
{
  mTable->setTableData(tableData);
  mBreakPointLookup->setBreakPointVector(lookup);
}

} //namespace OpenFDM
