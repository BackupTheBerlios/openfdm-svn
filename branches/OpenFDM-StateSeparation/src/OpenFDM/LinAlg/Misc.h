/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinAlgMisc_H
#define OpenFDM_LinAlgMisc_H

// #define USE_EXPRESSIONS

namespace OpenFDM {

namespace LinAlg {

// The common size_type for all the matrix/vector operations.
typedef unsigned size_type;

// Used to define an index range.
struct Range {
  OpenFDM_FORCE_INLINE
  Range(size_type f, size_type l)
    : first(f), last(l)
  {}
  OpenFDM_FORCE_INLINE
  Range(size_type one)
    : first(one), last(one)
  {}
  size_type first;
  size_type last;
};

// This class is having only one static function.
// This function checks if the dimensions in vector operations are compatible.
// The gerenic imlpementation tells is fir the case where s1 and s2 are both
// not equal to zero and are different. In this case we have an error.
template<size_type s1, size_type s2>
class SizeCheck;

// Sizes are equal, so nothing to do here.
template<size_type s>
struct SizeCheck<s,s> {
  static OpenFDM_FORCE_INLINE
  void
  Equal(size_type, size_type)
  { }
};

// Now the three cases where we need to check dynamically.
template<size_type s>
struct SizeCheck<0,s> {
  static OpenFDM_FORCE_INLINE
  void
  Equal(size_type s1, size_type s2)
  {
    if (s1 != s2)
      OpenFDMError("Incompatible dimensions in vector operations!");
  }
};

template<size_type s>
struct SizeCheck<s,0> {
  static OpenFDM_FORCE_INLINE
  void
  Equal(size_type s1, size_type s2)
  {
    if (s1 != s2)
      OpenFDMError("Incompatible dimensions in vector operations!");
  }
};

template<>
struct SizeCheck<0,0> {
  static OpenFDM_FORCE_INLINE
  void
  Equal(size_type s1, size_type s2)
  {
    if (s1 != s2)
      OpenFDMError("Incompatible dimensions in vector operations!");
  }
};

} // namespace LinAlg

// Make them available.
using LinAlg::Range;

} // namespace OpenFDM

#endif
