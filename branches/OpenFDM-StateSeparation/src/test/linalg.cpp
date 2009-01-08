/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include <OpenFDM/Quaternion.h>
#include <OpenFDM/Matrix.h>
#include <OpenFDM/Vector.h>

namespace OpenFDM {

/// Return random vector
Vector
rVec(unsigned len)
{
  Vector v(len);
  for (unsigned i = 0; i < len; ++i)
    v(i) = drand48() - real_type(0.5);
  return v;
}

/// Return normalized random vector
Vector
rnVec(unsigned len)
{
  return normalize(rVec(len));
}

Matrix
rMatrix(const Size& s)
{
  Matrix m(s(0), s(1));
  for (unsigned i = 0; i < s(0); ++i)
    for (unsigned j = 0; j < s(1); ++j)
      m(i, j) = drand48() - real_type(0.5);
  return m;
}

template<unsigned n>
bool
solvetest(const LinAlg::Matrix<real_type,n,n>& m,
          const LinAlg::Vector<real_type,n>& v)
{
  // compute a decomposition
  LinAlg::MatrixFactors<real_type,n,n,LinAlg::LUTag> decomp(m);

  // crude condition estimation
  real_type nrm = 0;
  real_type rnrm = 0;
  for (unsigned i = 0; i < rows(v); ++i) {
    nrm = max(nrm, norm1(m*Vector::unit(i, rows(v))));
    rnrm = max(rnrm, norm1(decomp.solve(Vector::unit(i, rows(v)))));
  }
  real_type cond = nrm*rnrm;

  // determine allowed tolerance from the condition number
  real_type rtol = sqrt(rows(v))*cond*10*Limits<real_type>::epsilon();
  real_type atol = sqrt(rows(v))*1e-5*Limits<real_type>::epsilon();

  // Check ...
  if (!equal(v, decomp.solve(m*v), rtol, atol)) {
    std::cerr << "Matrix solve test failed\n";
    std::cerr << "   condition number:       " << cond << "\n";
    std::cerr << "   norm of the difference: "
              << norm(v - decomp.solve(m*v)) << std::endl;
    return false;
  }

  return true;
}

template<unsigned n>
bool
solvetestFixedSize(void)
{
  Size s(n, n);
  for (unsigned i = 0; i < 100; ++i)
    if (!solvetest(rMatrix(s), rVec(s(0))))
      return false;
  return true;
}

bool
solvetestVariableSize(unsigned n)
{
  Size s(n, n);
  for (unsigned i = 0; i < 100; ++i)
    if (!solvetest(rMatrix(s), rnVec(s(0))))
      return false;
  return true;
}

bool
solvetest()
{
  if (!solvetestFixedSize<1>())
    return false;
  if (!solvetestFixedSize<2>())
    return false;
  if (!solvetestFixedSize<3>())
    return false;
  if (!solvetestFixedSize<4>())
    return false;
  if (!solvetestFixedSize<5>())
    return false;
  if (!solvetestFixedSize<6>())
    return false;
  for (unsigned n = 1; n < 50; ++n)
    if (!solvetestVariableSize(n))
      return false;
  return true;
}

}

int
main(int argc, char *argv[])
{
  if (!OpenFDM::solvetest())
    return EXIT_FAILURE;
  return EXIT_SUCCESS;
}
