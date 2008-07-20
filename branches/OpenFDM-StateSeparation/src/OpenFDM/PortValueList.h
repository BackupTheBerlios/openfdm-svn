/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortValueList_H
#define OpenFDM_PortValueList_H

#include "PortValue.h"
#include "MatrixInputPort.h"
#include "MatrixOutputPort.h"
#include "RealInputPort.h"
#include "RealOutputPort.h"

namespace OpenFDM {

// A set of port values that a Model gets in its methods.
// Access control to the way the values are accessed is done through the
// Port type system and this class. So do not add additional accessors to the
// port values unless you know what you are doing ...
// Many methods from this class are in the fast path of the simulation, so
// this methods code paths might be more typesafe implemented with virtuals,
// but this implementation without and partly with static casts is used to
// avoid calling many virtual fuctions in the fast path.
class PortValueList {
public:

  // Accessors for real valued ports
  const real_type& getValue(const RealInputPort& port) const
  { return port.getPortValue(mPortValueVector)->getValue()(0, 0); }
  void setValue(const RealOutputPort& port, const real_type& value)
  { port.getPortValue(mPortValueVector)->getValue()(0, 0) = value; }

  const real_type& operator[](const RealInputPort& port) const
  { return port.getPortValue(mPortValueVector)->getValue()(0, 0); }
  real_type& operator[](const RealOutputPort& port)
  { return port.getPortValue(mPortValueVector)->getValue()(0, 0); }

  // Accessors for matrix valued ports
  const Matrix& getValue(const MatrixInputPort& port) const
  { return port.getPortValue(mPortValueVector)->getValue(); }
  void setValue(const MatrixOutputPort& port, const Matrix& matrix)
  { port.getPortValue(mPortValueVector)->setValue(matrix); }
  // FIXME, will have them, but cannot ensure currently that the size does not change.
  const Matrix& operator[](const MatrixInputPort& port) const
  { return port.getPortValue(mPortValueVector)->getValue(); }
//   Matrix& operator[](const MatrixOutputPort& port)
//   { return port.getPortValue(mPortValueVector)->getValue(); }

  bool isConnected(const RealOutputPort& port) const
  { return port.getPortValue(mPortValueVector); }
  bool isConnected(const MatrixOutputPort& port) const
  { return port.getPortValue(mPortValueVector); }

  void setPortValue(unsigned idx, PortValue* portValue) // FIXME
  {
    if (mPortValueVector.size() <= idx)
      mPortValueVector.resize(idx+1);
    mPortValueVector[idx] = portValue;
  }

private:
  PortValueVector mPortValueVector;
};

} // namespace OpenFDM

#endif
