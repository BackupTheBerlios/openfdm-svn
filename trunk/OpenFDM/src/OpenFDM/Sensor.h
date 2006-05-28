/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Sensor_H
#define OpenFDM_Sensor_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Gravity.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Environment.h"
#include "Interact.h"

namespace OpenFDM {

class Sensor
  : public Interact {
public:
  Sensor(const std::string& name) :
    Interact(name, 1)
  {
    setNumOutputPorts(2);
    setOutputPort(0, "nlfz", this, &Sensor::getNlfz);
    setOutputPort(1, "az", this, &Sensor::getAz);
  }
  virtual ~Sensor(void)
  { }

  virtual bool init(void)
  {
    mNextNlfz = 0;
    mNextAz = 0;
    return Interact::init();
  }

  virtual void output(const TaskInfo& taskInfo)
  {
    mNlfz = mNextNlfz;
    mAz = mNextAz;
  }

  virtual void update(const TaskInfo& taskInfo)
  {
    if (!nonZeroIntersection(taskInfo.getSampleTimeSet(), getSampleTimeSet()))
        return;

    mNextNlfz = 0;
    mNextAz = 0;
    RigidBody* rigidBody = getParentRigidBody(0);
    if (!rigidBody)
      return;
    Frame* frame = rigidBody->getFrame();
    if (!frame)
      return;
    const Gravity* grav = mEnvironment->getGravity();
    if (!grav)
      return;
    Vector3 accel = frame->getClassicAccel().getLinear();
    // That is the acceleration like sensed by a gyro
    mNextAz = accel(3);

    // Now compute the acceleration like sensed by anything sensing the
    // gravitational stuff too
    accel -= frame->rotFromRef(grav->gravityAccel(frame->getRefPosition()));
    mNextNlfz = accel(3)/9.81;
  }

  const real_type& getNlfz(void) const
  { return mNlfz; }
  const real_type& getAz(void) const
  { return mAz; }

  virtual void interactWith(RigidBody*)
  {}

protected:
  virtual void setEnvironment(Environment* environment)
  { mEnvironment = environment; }

private:
  SharedPtr<Environment> mEnvironment;
  real_type mNlfz;
  real_type mNextNlfz;
  real_type mAz;
  real_type mNextAz;
};

} // namespace OpenFDM

#endif
