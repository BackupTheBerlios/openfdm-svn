/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Product.h"

#include <string>
#include "Types.h"
#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Product, Model)
  DEF_OPENFDM_PROPERTY(Unsigned, NumFactors, Serialized)
  END_OPENFDM_OBJECT_DEF

Product::Product(const std::string& name) :
  Model(name)
{
  setDirectFeedThrough(true);
  setNumInputPorts(2);
  setInputPortName(0, "*");
  setInputPortName(1, "*");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &Product::getProduct);
}

Product::~Product(void)
{
}

bool
Product::init(void)
{
  mScalarFactorPorts.clear();
  mMatrixFactorPorts.clear();
  for (unsigned i = 0; i < getNumInputPorts(); ++i) {
    RealPortHandle scalarHandle = getInputPort(i)->toRealPortHandle();
    if (scalarHandle.isConnected())
      mScalarFactorPorts.push_back(scalarHandle);
    else {
      MatrixPortHandle matrixHandle = getInputPort(i)->toMatrixPortHandle();
      if (matrixHandle.isConnected()) {
        if (!mMatrixFactorPorts.empty()) {
          unsigned lastCols = cols(mMatrixFactorPorts.back().getMatrixValue());
          unsigned thisRows = rows(matrixHandle.getMatrixValue());
          if (lastCols != thisRows) {
            Log(Model, Error) << "Dimensions for Product \""
                              << getName() << "\" do not agree!" << endl;
            return false;
          }
        }

        mMatrixFactorPorts.push_back(matrixHandle);
      } else {
        Log(Model, Error) << "Found unconnected input Port for Product \""
                          << getName() << "\"" << endl;
        return false;
      }
    }
  }
  if (mMatrixFactorPorts.empty()) {
    mProduct.resize(1, 1);
  } else {
    mProduct.resize(rows(mMatrixFactorPorts.front().getMatrixValue()),
                    cols(mMatrixFactorPorts.back().getMatrixValue()));
  }

  return Model::init();
}

void
Product::output(const TaskInfo&)
{
  real_type scalarFac = 1;
  for (unsigned i = 0; i < mScalarFactorPorts.size(); ++i)
    scalarFac *= mScalarFactorPorts[i].getRealValue();
  if (mMatrixFactorPorts.empty()) {
    mProduct(1, 1) = scalarFac;
  } else {
    mProduct = mMatrixFactorPorts[0].getMatrixValue();
    for (unsigned i = 1; i < mMatrixFactorPorts.size(); ++i)
      mProduct = mProduct*mMatrixFactorPorts[i].getMatrixValue();
    mProduct *= scalarFac;
  }
  Log(Model,Debug3) << "Output of Product \"" << getName() << "\" "
                    << mProduct << endl;
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
Product::setNumFactors(unsigned num)
{
  unsigned oldnum = getNumFactors();
  setNumInputPorts(num);
  for (; oldnum < num; ++oldnum)
    setInputPortName(oldnum, "*");
}

} // namespace OpenFDM
