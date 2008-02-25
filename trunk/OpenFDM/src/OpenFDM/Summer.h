/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Summer_H
#define OpenFDM_Summer_H

#include <string>

#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

class Summer : public Model {
  OPENFDM_OBJECT(Summer, Model);
public:
  Summer(const std::string& name);
  virtual ~Summer(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const Matrix& getSum(void) const;

  unsigned getNumSummands(void) const;
  void setNumSummands(unsigned num);

  enum Sign { Plus, Minus };
  void setInputSign(unsigned num, Sign sign);
  Sign getInputSign(unsigned num) const;

private:
  Matrix mSum;
  std::vector<MatrixPortHandle> mPositiveSummandPorts;
  std::vector<MatrixPortHandle> mNegativeSummandPorts;
};

} // namespace OpenFDM

#endif
