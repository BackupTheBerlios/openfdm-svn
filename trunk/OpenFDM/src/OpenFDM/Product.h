/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Product_H
#define OpenFDM_Product_H

#include <string>

#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

class Product :
    public Model {
public:
  Product(const std::string& name);
  virtual ~Product(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getProduct(void) const;

  unsigned getNumFactors(void) const;
  void setNumFactors(const unsigned& num);

private:
  real_type mProduct;
  std::vector<RealPortHandle> mFactorPorts;
};

} // namespace OpenFDM

#endif
