/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include <vector>
#include <OpenFDM/Vector.h>
#include <OpenFDM/Output.h>

#ifndef ErrorCollectorCallback_H
#define ErrorCollectorCallback_H

class ErrorCollectorCallback : public OpenFDM::Output::Callback {
public:
  virtual void setValue(OpenFDM::real_type value)
  { values.push_back(value); }
  void print() const
  {
    std::vector<OpenFDM::real_type>::const_iterator i;
    for (i = values.begin(); i != values.end(); ++i)
      std::cout << *i << std::endl;
  }
  OpenFDM::real_type error() const
  {
    OpenFDM::Vector errors(values.size());
    for (unsigned i = 0; i < values.size(); ++i)
      errors(i) = values[i];
    return normInf(errors);
  }
private:
  std::vector<OpenFDM::real_type> values;
};

#endif
