/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include <OpenFDM/Quaternion.h>

namespace OpenFDM {

/// Return random vector
Vector3
rVec(void)
{
  return Vector3(drand48()-0.5, drand48()-0.5, drand48()-0.5);
}

/// Return normalized random vector
Vector3
rnVec(void)
{
  return normalize(rVec());
}

/// Return normalized random vector
Quaternion
rnQuat(void)
{
  Quaternion q(drand48()-0.5, drand48()-0.5, drand48()-0.5, drand48()-0.5);
  return Quaternion(normalize(q));
}

int
eulerTest(const Quaternion& q, real_type testEps)
{
  Vector3 euler = q.getEuler();
  Quaternion q2 = Quaternion::fromEuler(euler);
  Vector3 euler2 = q2.getEuler();
  
  if (!equal(q, q2, testEps) && !equal(q, -q2, testEps)) {
    std::cerr << "Failing on test Quaternion euler angles conversion:\n"
              << "q = " << q << "\n"
              << "q2 = " << q2
              << std::endl;
    return -1;
  }

  
  if (!equal(euler, euler2, testEps)) {
    std::cerr << "Failing on test Quaternion euler angles conversion:\n"
              << "q = " << q << "\n"
              << "q2 = " << q2
              << std::endl;
    return -1;
  }


  return 0;
}

int
quattest(void)
{
  real_type eps = Limits<real_type>::epsilon();
  unsigned nTests = 10000;

  // Testing fromRotateTo factory
  for (unsigned i = 0; i < nTests; ++i) {
    Vector3 from = rnVec();
    Vector3 to = rnVec();
    Quaternion q = Quaternion::fromRotateTo(from, to);

    if (!equal(q.transform(from), to, 1e4*eps)) {
      std::cerr << "Failing on test " << i << " Quaternion::fromRotateTo("
                << trans(from) << ", " << trans(to) << "):\n"
                << "q = " << q << "\n"
                << "q.transform(from) = " << trans(q.transform(from)) << "\n"
                << "q.backTransform(from) = " << trans(q.backTransform(from))
                << std::endl;
      return -1;
    }
  }

  // Testing fromRotateTo factory
  for (unsigned i = 0; i < nTests; ++i) {
    Vector3 from1 = rnVec();
    unsigned i1 = 0;
    Vector3 from2 = rnVec();
    unsigned i2 = 1;
    // Make sure they are not linearily dependent
    while (dot(from1, from2) < 0.1) {
      from2 = rnVec();
    }

    Quaternion q = Quaternion::fromRotateTo(from1, i1, from2, i2);

    if (!equal(q.transform(from1), Vector3::unit(i1), 1e4*eps)) {
      std::cerr << "Failing on test " << i << " Quaternion::fromRotateTo("
                << trans(from1) << ", " << i1 << ", "
                << trans(from2) << ", " << i2 << "):\n"
                << "q = " << q << "\n"
                << "q.transform(from1) = " << trans(q.transform(from1)) << "\n"
                << "q.backTransform(from1) = " << trans(q.backTransform(from1))
                << std::endl;
      return -1;
    }
    Vector3 from2Orth = normalize(cross(cross(from1, from2), from1));
    if (!equal(q.transform(from2Orth), Vector3::unit(i2), 1e4*eps)
        || (0 > dot(q.transform(from2), Vector3::unit(i2)))) {
      std::cerr << "Failing on test " << i << " Quaternion::fromRotateTo("
                << trans(from1) << ", " << i1 << ", "
                << trans(from2) << ", " << i2 << "):\n"
                << "q = " << q << "\n"
                << "q.transform(from2Orth) = "
                << trans(q.transform(from2Orth)) << "\n"
                << "q.backTransform(from2Orth) = "
                << trans(q.backTransform(from2Orth))
                << std::endl;
      return -1;
    }
  }

  // Test euler angle to quaternion and back conversion.
  // special fixed cases
  int fail = 0;
  fail += eulerTest(Quaternion::fromEuler(0, 0, 0), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(pi05, 0, 0), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(pi05, 0, pi05), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(0, 0, pi05), 100*eps);
  // special fixed cases at the gimbal lock
  fail += eulerTest(Quaternion::fromEuler(0, pi05, 0), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(pi05, pi05, 0), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(pi05, pi05, pi05), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(0, pi05, pi05), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(0, -pi05, 0), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(pi05, -pi05, 0), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(pi05, -pi05, pi05), 100*eps);
  fail += eulerTest(Quaternion::fromEuler(0, -pi05, pi05), 100*eps);

  real_type gimbalEps = 1e3*eps;
  // special cases around the gimbal lock
  for (unsigned i = 0; i < nTests; ++i) {
    for (unsigned k = 1; k < 1024; k *= 2) {
      Vector3 euler = rVec();
      euler(1) = pi05;
      fail += eulerTest(Quaternion::fromEuler(euler), gimbalEps);

      euler(1) = pi05*(1 - k*Limits<real_type>::epsilon());
      fail += eulerTest(Quaternion::fromEuler(euler), gimbalEps);

      euler(1) = pi05*(1 + k*Limits<real_type>::epsilon());
      fail += eulerTest(Quaternion::fromEuler(euler), gimbalEps);

      euler(1) = -pi05;
      fail += eulerTest(Quaternion::fromEuler(euler), gimbalEps);

      euler(1) = -pi05*(1 - k*Limits<real_type>::epsilon());
      fail += eulerTest(Quaternion::fromEuler(euler), gimbalEps);

      euler(1) = -pi05*(1 + k*Limits<real_type>::epsilon());
      fail += eulerTest(Quaternion::fromEuler(euler), gimbalEps);
    }
  }
  // arbitrary cases
  for (unsigned i = 0; i < nTests; ++i)
    fail += eulerTest(rnQuat(), 100*eps);

  if (fail)
    return -1;

  return 0;
}

}

int
main(int argc, char *argv[])
{
  return OpenFDM::quattest();
}
