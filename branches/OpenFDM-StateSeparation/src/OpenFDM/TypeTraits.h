/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TypeTraits_H
#define OpenFDM_TypeTraits_H

namespace OpenFDM {

template<typename S, typename T>
struct TypeTraits;

template<typename T>
struct TypeTraits<T,T> {
  typedef T result_type;
  typedef T limits_type;
};

template<>
struct TypeTraits<float,double> {
  typedef double result_type;
  typedef float limits_type;
};
template<>
struct TypeTraits<double,float> {
  typedef double result_type;
  typedef float limits_type;
};
template<>
struct TypeTraits<float,long double> {
  typedef long double result_type;
  typedef float limits_type;
};
template<>
struct TypeTraits<long double,float> {
  typedef long double result_type;
  typedef float limits_type;
};
template<>
struct TypeTraits<double,long double> {
  typedef long double result_type;
  typedef double limits_type;
};
template<>
struct TypeTraits<long double,double> {
  typedef long double result_type;
  typedef double limits_type;
};

}

#endif
