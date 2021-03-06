/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Product_H
#define OpenFDM_Product_H

#include <string>

#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

class Product : public Model {
  OPENFDM_OBJECT(Product, Model);
public:
  Product(const std::string& name);
  virtual ~Product(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const Matrix& getProduct(void) const;

  unsigned getNumFactors(void) const;
  void setNumFactors(unsigned num);

private:
  Matrix mProduct;
  std::vector<RealPortHandle> mScalarFactorPorts;
  std::vector<MatrixPortHandle> mMatrixFactorPorts;
};

} // namespace OpenFDM

#endif
