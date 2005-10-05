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
  setOutputPort(0, "output", Property(this, &Product::getProduct));
  addProperty("output", Property(this, &Product::getProduct));

  addProperty("numFactors", Property(this, &Product::getNumFactors, &Product::setNumFactors));
}

Product::~Product(void)
{
}

bool
Product::init(void)
{
  for (unsigned i = 0; i < getNumInputPorts(); ++i)
    OpenFDMAssert(getInputPort(i).isValid());
  
  // Make sure it is invalid if sizes do not match.
  mProduct.resize(0, 0);
  // Check if the sizes match.
  for (unsigned i = 0; i < getNumInputPorts(); ++i) {
    Matrix a = getInputPort(i).getValue().toMatrix();
    if (Size(1,1) != size(a))
      return false;
  }
  mProduct.resize(1, 1);
  mProduct.resize(getInputPort(0).getValue().toMatrix());
  return true;
}

void
Product::output(void)
{
  mProduct = getInputPort(0).getValue().toMatrix();
  for (unsigned i = 1; i < getNumInputPorts(); ++i) {
    Matrix a = getInputPort(i).getValue().toMatrix();
    if (getInputPortName(i) == "*")
      mProduct(1,1) *= a(1,1);
    else
      mProduct(1,1) /= a(1,1);
  }
}

const Matrix&
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
