/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Property.h"
#include "Model.h"
#include "Product.h"

namespace OpenFDM {

Product::Product(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  setNumInputPorts(2);
  setInputPortName(0, "*");
  setInputPortName(1, "*");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Product::getProduct);

  addProperty("output", Property(this, &Product::getProduct));
  addProperty("numFactors", Property(this, &Product::getNumFactors, &Product::setNumFactors));
}

Product::~Product(void)
{
}

bool
Product::init(void)
{
  mFactorPorts.resize(getNumFactors());
  for (unsigned i = 0; i < getNumInputPorts(); ++i) {
    if (!getInputPort(i)->isConnected()) {
      Log(Model, Error) << "Found unconnected input Port for Product \""
                        << getName() << "\"" << endl;
      return false;
    }
    mFactorPorts[i] = getInputPort(i)->toRealPortHandle();
  }

  return true;
}

void
Product::output(const TaskInfo&)
{
  mProduct = 1;
  for (unsigned i = 0; i < getNumInputPorts(); ++i)
    mProduct *= mFactorPorts[i].getRealValue();
}

const real_type&
Product::getProduct(void) const
{
  return mProduct;
}

unsigned
Product::getNumFactors(void) const
{
  return getNumInputPorts();
}

void
Product::setNumFactors(const unsigned& num)
{
  unsigned oldnum = getNumFactors();
  setNumInputPorts(num);
  for (; oldnum < num; ++oldnum)
    setInputPortName(oldnum, "*");
}

} // namespace OpenFDM
