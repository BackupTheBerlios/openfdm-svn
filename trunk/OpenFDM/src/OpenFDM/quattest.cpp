/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include "Quaternion.h"

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

    if (!equal(q.transform(from), to, 100*eps)) {
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
    unsigned i1 = 1;
    Vector3 from2 = rnVec();
    unsigned i2 = 2;
    // Make sure they are not linearily dependent
    while (dot(from1, from2) < 0.1) {
      from2 = rnVec();
    }

    Quaternion q = Quaternion::fromRotateTo(from1, i1, from2, i2);

    if (!equal(q.transform(from1), Vector3::unit(i1), 1e2*eps)) {
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
    if (!equal(q.transform(from2Orth), Vector3::unit(i2), 1e3*eps)
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

  return 0;
}

}

int
main(int argc, char *argv[])
{
  return OpenFDM::quattest();
}
