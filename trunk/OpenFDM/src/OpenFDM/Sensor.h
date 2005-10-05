/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Sensor_H
#define OpenFDM_Sensor_H

#include "Assert.h"
#include "Object.h"
#include "Node.h"
#include "Group.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"

namespace OpenFDM {

class Sensor
  : public Node {
  OpenFDM_NodeImplementation(2);
public:
  Sensor(void) {}
  virtual ~Sensor(void) {}

  Frame* getParentFrame(unsigned idx)
  {
    Group* g = getParent(idx);
    if (g)
      return g->toFrame();
    else
      return 0;
  }
  const Frame* getParentFrame(unsigned idx) const
  {
    const Group* g = getParent(idx);
    if (g)
      return g->toFrame();
    else
      return 0;
  }

  bool setPosition(unsigned frameIdx, const Vector3& pos)
  {
    if (NumberOfParents <= frameIdx)
      return false;

    mPosition[frameIdx] = pos;
    return true;
  }

  const Vector3& getPosition(unsigned frameIdx) const
  {
    // FIXME, somehow wrong ...
    if (NumberOfParents <= frameIdx)
      return mPosition[0];
    return mPosition[frameIdx];
  }

  Vector3 getOffset(unsigned frameIdx) const
  {
    if (frameIdx == 0) {
      const Frame* f0 = getParentFrame(0);
      const Frame* f1 = getParentFrame(1);
      // FIXME, somehow wrong ...
      if (!f0 || !f1)
        return Vector3::zeros();
      
      return f1->posFromRef(f0->posToRef(mPosition[0])) - mPosition[1];
    } else if (frameIdx == 1) {
      const Frame* f0 = getParentFrame(0);
      const Frame* f1 = getParentFrame(1);
      // FIXME, somehow wrong ...
      if (!f0 || !f1)
        return Vector3::zeros();
      
      return f0->posFromRef(f1->posToRef(mPosition[1])) - mPosition[0];
    } else
      return Vector3::zeros();
  }

  real_type getDistance(void) const
  { return norm(getOffset(0)); }

private:
  Vector3 xxx(const Frame* f0, const Frame* f1,
              const Vector3& p0)
  { return f1->posFromRef(f0->posToRef(p0)); }

  

  Vector3 mPosition[NumberOfParents];
//   Vector3 mPosition[NumberOfParents];

};

} // namespace OpenFDM

#endif
