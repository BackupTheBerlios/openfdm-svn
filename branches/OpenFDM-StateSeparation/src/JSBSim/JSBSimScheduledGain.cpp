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
  getGroup()->connect(product->getPort("input0"),
                      mTable->getPort("output"));
  getGroup()->connect(mTable->getPort("input"),
                      mBreakPointLookup->getPort("output"));

  // Now connect the input and the output to this groups in and outputs
  GroupInput* groupInput = new GroupInput("Input");
  getGroup()->addChild(groupInput);
  getGroup()->connect(product->getPort("input1"),
                      groupInput->getPort("output"));
 
  groupInput = new GroupInput("Schedule Input");
  getGroup()->addChild(groupInput);
  getGroup()->connect(mBreakPointLookup->getPort("input"),
                      groupInput->getPort("output"));

  // That single output port is this one
  getGroup()->connect(getInternalOutputPort(),
                      product->getPort("output"));
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
