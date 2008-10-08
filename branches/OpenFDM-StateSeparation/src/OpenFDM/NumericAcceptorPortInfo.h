/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericAcceptorPortInfo_H
#define OpenFDM_NumericAcceptorPortInfo_H

#include <string>
#include "AcceptorPortInfo.h"
#include "Matrix.h"
#include "NumericPortValue.h"

namespace OpenFDM {

class NumericAcceptorPortInfo : public AcceptorPortInfo {
public:
  NumericAcceptorPortInfo(Node* node, const std::string& name,
                          const Size& size, bool directInput);
  virtual ~NumericAcceptorPortInfo();

  virtual bool getDirectInput() const
  { return mDirectInput; }
  void setDirectInput(bool directInput)
  { mDirectInput = directInput; }

  virtual unsigned getMaxConnects() const
  { return 1; }

  virtual bool acceptPortValue(const PortValue* portValue) const
  { return dynamic_cast<const NumericPortValue*>(portValue); }

private:
  Size mSize;
  bool mDirectInput;
};

} // namespace OpenFDM

#endif
