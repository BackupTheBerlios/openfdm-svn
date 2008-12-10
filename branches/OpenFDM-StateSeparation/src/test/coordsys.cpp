/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include <iostream>
#include <OpenFDM/CoordinateSystem.h>
#include <OpenFDM/Quaternion.h>
#include <OpenFDM/Matrix.h>
#include <OpenFDM/Rotation.h>
#include <OpenFDM/Transform.h>
#include <OpenFDM/Vector.h>

namespace OpenFDM {

class ReferenceFrame {
public:
  ReferenceFrame() :
    mSpatialVelocity(Vector6::zeros()),
    mSpatialAcceleration(Vector6::zeros())
  { }
  ReferenceFrame(const ReferenceFrame& referenceFrame) :
    mSpatialVelocity(referenceFrame.mSpatialVelocity),
    mSpatialAcceleration(referenceFrame.mSpatialAcceleration)
  { }

  const Vector6& getSpatialVelocity() const
  { return mSpatialVelocity; }
  void setSpatialVelocity(const Vector6& spatialVelocity)
  { mSpatialVelocity = spatialVelocity; }

  const Vector6& getSpatialAcceleration() const
  { return mSpatialAcceleration; }
  void setSpatialAcceleration(const Vector6& spatialAcceleration)
  { mSpatialAcceleration = spatialAcceleration; }

  CoordinateSystem mCoordinateSystem; /// ????
private:
  Vector6 mSpatialVelocity;
  Vector6 mSpatialAcceleration;
};

// LinkValue has a root coordinate system and a chain of them???

}

using namespace OpenFDM;

int
main(int argc, char *argv[])
{
  CoordinateSystem cs(Vector3(1, 0, 0));
  CoordinateSystem cs2 = cs.getRelative(Vector3(1, 0, 0), Quaternion::fromEuler(Vector3(0, pi/2, 0)));
  CoordinateSystem cs3 = cs2.getRelative(Vector3(1, 0, 0));

  std::cout << cs << std::endl;
  std::cout << cs2 << std::endl;
  std::cout << cs3 << std::endl;
  std::cout << cs3.referenceToLocal() << std::endl;

  std::cout << trans(cs.toReference(Vector3(1, 1, 1))) << std::endl;
  std::cout << trans(cs2.toReference(Vector3(1, 1, 1))) << std::endl;
  std::cout << trans(cs3.toReference(Vector3(1, 1, 1))) << std::endl;

  std::cout << trans(cs.referenceToLocal().toReference(cs.toReference(Vector3(1, 1, 1)))) << std::endl;
  std::cout << trans(cs2.referenceToLocal().toReference(cs2.toReference(Vector3(1, 1, 1)))) << std::endl;
  std::cout << trans(cs3.referenceToLocal().toReference(cs3.toReference(Vector3(1, 1, 1)))) << std::endl;

  std::cout << trans(cs.toLocal(cs.toReference(Vector3(1, 1, 1)))) << std::endl;
  std::cout << trans(cs2.toLocal(cs2.toReference(Vector3(1, 1, 1)))) << std::endl;
  std::cout << trans(cs3.toLocal(cs3.toReference(Vector3(1, 1, 1)))) << std::endl;

  return EXIT_SUCCESS;
}
