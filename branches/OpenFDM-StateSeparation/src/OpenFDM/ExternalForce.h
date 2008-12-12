/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ExternalForce_H
#define OpenFDM_ExternalForce_H

#include "Sensor.h"
#include "MatrixInputPort.h"

namespace OpenFDM {

class ExternalForce : public Sensor {
  OPENFDM_OBJECT(ExternalForce, Sensor);
public:
  ExternalForce(const std::string& name);
  virtual ~ExternalForce(void);

  virtual void articulation(const Task&, const Environment&,
                            const ContinousStateValueVector&,
                            PortValueList& portValues) const;

  void setLocalCoordinates(bool localCoordinates);
  bool getLocalCoordinates() const;

private:
  MatrixInputPort mForcePort;
  bool mLocalCoordinates;
};

} // namespace OpenFDM

#endif
