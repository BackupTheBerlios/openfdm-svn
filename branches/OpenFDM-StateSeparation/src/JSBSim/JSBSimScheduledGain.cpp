/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
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
  getGroup()->connect(product->getInputPort(0),
                      mTable->getOutputPort());
  getGroup()->connect(mTable->getInputPort(0),
                      mBreakPointLookup->getOutputPort());

  // Now connect the input and the output to this groups in and outputs
  GroupInput* groupInput = new GroupInput("Input");
  getGroup()->addChild(groupInput);
  groupInput->setExternalPortName("input");
  getGroup()->connect(product->getInputPort(1),
                      groupInput->getPort("output"));
 
  groupInput = new GroupInput("Schedule Input");
  getGroup()->addChild(groupInput);
  groupInput->setExternalPortName("scheduleInput");
  getGroup()->connect(mBreakPointLookup->getInputPort(0),
                      groupInput->getPort("output"));

  // That single output port is this one
  getGroup()->connect(getInternalOutputPort(),
                      product->getOutputPort());
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
