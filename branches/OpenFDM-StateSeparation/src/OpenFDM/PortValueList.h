/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortValueList_H
#define OpenFDM_PortValueList_H

#include "PortValue.h"
#include "MatrixInputPort.h"
#include "MatrixOutputPort.h"
#include "MechanicBodyPort.h"
#include "MechanicInteractPort.h"
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
  bool isConnected(const RealInputPort& port) const
  { return port.getPortValue(mPortValueVector); }
  bool isConnected(const RealOutputPort& port) const
  { return port.getPortValue(mPortValueVector); }

  const real_type& operator[](const RealInputPort& port) const
  { return port.getPortValue(mPortValueVector)->getValue()(0, 0); }
  real_type& operator[](const RealOutputPort& port)
  { return port.getPortValue(mPortValueVector)->getValue()(0, 0); }

  // Accessors for matrix valued ports
  bool isConnected(const MatrixInputPort& port) const
  { return port.getPortValue(mPortValueVector); }
  bool isConnected(const MatrixOutputPort& port) const
  { return port.getPortValue(mPortValueVector); }

  const Matrix& operator[](const MatrixInputPort& port) const
  { return port.getPortValue(mPortValueVector)->getValue(); }
  // FIXME, make sure that the size cannot change.
  // May be we have some kind of base class of Matrix that has no resize call
  // and no resizing assignment/copy???
  Matrix& operator[](const MatrixOutputPort& port)
  { return port.getPortValue(mPortValueVector)->getValue(); }


  // Accessors for matrix valued ports
  bool isConnected(const MechanicBodyPort& port) const
  { return port.getPortValue(mPortValueVector); }
  bool isConnected(const MechanicInteractPort& port) const
  { return port.getPortValue(mPortValueVector); }

  // FIXME Implement access control for the port value
  MechanicPortValue& operator[](const MechanicBodyPort& port) const
  { return *port.getPortValue(mPortValueVector); }
  MechanicPortValue& operator[](const MechanicInteractPort& port)
  { return *port.getPortValue(mPortValueVector); }




  // FIXME, avoid this method here. With this method the output stage of a model
  // can change the port values, this should not be available in a model.
  // may be this must be a derived class that provides some more access??
  void setPortValue(unsigned idx, PortValue* portValue)
  {
    if (mPortValueVector.size() <= idx)
      mPortValueVector.resize(idx+1);
    mPortValueVector[idx] = portValue;
  }
  void setPortSize(const MatrixOutputPort& port, const Size& size)
  {
    setPortValue(port.getPortIndex(), new NumericPortValue(size));
  }
  const PortValue* getPortValue(unsigned idx) const
  {
    if (mPortValueVector.size() <= idx)
      return 0;
    return mPortValueVector[idx];
  }

protected:
  PortValueVector mPortValueVector;
};

} // namespace OpenFDM

#endif
