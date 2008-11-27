/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Product_H
#define OpenFDM_Product_H

#include <string>

#include "SimpleDirectModel.h"

namespace OpenFDM {

class Product : public SimpleDirectModel {
  OPENFDM_OBJECT(Product, SimpleDirectModel);
public:
  Product(const std::string& name);
  virtual ~Product(void);

  // FIXME implements only pointwise products. Also want matrix products

  virtual void output(Context& context) const;

  unsigned getNumFactors(void) const;
  void setNumFactors(unsigned num);
};

} // namespace OpenFDM

#endif
