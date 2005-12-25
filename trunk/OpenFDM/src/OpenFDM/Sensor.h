/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Sensor_H
#define OpenFDM_Sensor_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Visitor.h"
#include "ConstVisitor.h"
#include "MultiBodyModel.h"

namespace OpenFDM {

class Sensor
  : public MultiBodyModel {
public:
  Sensor(const std::string& name) :
    MultiBodyModel(name)
  {
    mAccel.resize(6, 1);
    setNumOutputPorts(1);
    setOutputPort(0, "nz", this, &Sensor::getNZ);
  }
  virtual ~Sensor(void)
  { }

  virtual void accept(Visitor& visitor)
  { visitor.apply(*this); }
  virtual void accept(ConstVisitor& visitor) const
  { visitor.apply(*this); }

  void output(const TaskInfo& taskInfo)
  {
    if (!nonZeroIntersection(taskInfo.getSampleTimeSet(), getSampleTimeSet()))
        return;

    Frame* frame = getParentFrame(0);
    if (!frame) {
      mAccel = Vector6::zeros();
      return;
    }
    mAccel = frame->getClassicAccel();
    mNz = mAccel(6, 1)/9.81;
  }

  const real_type& getNZ(void) const
  { return mNz; }

private:
  OpenFDM_NodeImplementation(1);

  Matrix mAccel;
  real_type mNz;
};

} // namespace OpenFDM

#endif
