/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MPFloat_H
#define OpenFDM_MPFloat_H

#include <cstring>
#include <iosfwd>
#include <ostream>
#include <mpfr.h>

namespace OpenFDM {

/// Class for a floating point type that uses mpfr with an fixed but huge
/// precision. This is just for doing test of the numeric included algorithms.
/// Never use that for realtime simulations ...

class MPFRFloat;

#define _BINARY_OPERATOR_DECL(qualifier, ret, op)                           \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const MPFRFloat& value2);              \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const float& value2);                  \
qualifier ret                                                               \
operator op(const float& value1, const MPFRFloat& value2);                  \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const double& value2);                 \
qualifier ret                                                               \
operator op(const double& value1, const MPFRFloat& value2);                 \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const long double& value2);            \
qualifier ret                                                               \
operator op(const long double& value1, const MPFRFloat& value2);            \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const signed char& value2);            \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const signed short& value2);           \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const signed int& value2);             \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const signed long& value2);            \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const unsigned char& value2);          \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const unsigned short& value2);         \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const unsigned int& value2);           \
qualifier ret                                                               \
operator op(const MPFRFloat& value1, const unsigned long& value2);          \
qualifier ret                                                               \
operator op(const signed char& value1, const MPFRFloat& value2);            \
qualifier ret                                                               \
operator op(const signed short& value1, const MPFRFloat& value2);           \
qualifier ret                                                               \
operator op(const signed int& value1, const MPFRFloat& value2);             \
qualifier ret                                                               \
operator op(const signed long& value1, const MPFRFloat& value2);            \
qualifier ret                                                               \
operator op(const unsigned char& value1, const MPFRFloat& value2);          \
qualifier ret                                                               \
operator op(const unsigned short& value1, const MPFRFloat& value2);         \
qualifier ret                                                               \
operator op(const unsigned int& value1, const MPFRFloat& value2);           \
qualifier ret                                                               \
operator op(const unsigned long& value1, const MPFRFloat& value2);

_BINARY_OPERATOR_DECL(inline,MPFRFloat,+)
_BINARY_OPERATOR_DECL(inline,MPFRFloat,-)
_BINARY_OPERATOR_DECL(inline,MPFRFloat,*)
_BINARY_OPERATOR_DECL(inline,MPFRFloat,/)
_BINARY_OPERATOR_DECL(inline,bool,==)
_BINARY_OPERATOR_DECL(inline,bool,!=)
_BINARY_OPERATOR_DECL(inline,bool,<)
_BINARY_OPERATOR_DECL(inline,bool,<=)
_BINARY_OPERATOR_DECL(inline,bool,>)
_BINARY_OPERATOR_DECL(inline,bool,>=)

#define _FUNCTIONS_DECL(qualifier)                                           \
qualifier MPFRFloat abs(const MPFRFloat& value);                             \
qualifier MPFRFloat acos(const MPFRFloat& value);                            \
qualifier MPFRFloat asin(const MPFRFloat& value);                            \
qualifier MPFRFloat atan(const MPFRFloat& value);                            \
qualifier MPFRFloat atan2(const MPFRFloat& value1, const MPFRFloat& value2); \
qualifier MPFRFloat ceil(const MPFRFloat& value);                            \
qualifier MPFRFloat cos(const MPFRFloat& value);                             \
qualifier MPFRFloat cosh(const MPFRFloat& value);                            \
qualifier MPFRFloat exp(const MPFRFloat& value);                             \
qualifier MPFRFloat fabs(const MPFRFloat& value);                            \
qualifier MPFRFloat floor(const MPFRFloat& value);                           \
qualifier MPFRFloat fmod(const MPFRFloat& value1, const MPFRFloat& value2);  \
qualifier MPFRFloat frexp(const MPFRFloat& value1, int * value2);            \
qualifier MPFRFloat ldexp(const MPFRFloat& value1, int value2);              \
qualifier MPFRFloat log(const MPFRFloat& value);                             \
qualifier MPFRFloat log10(const MPFRFloat& value);                           \
qualifier MPFRFloat pow(const MPFRFloat& value1, const MPFRFloat& value2);   \
qualifier MPFRFloat sin(const MPFRFloat& value);                             \
qualifier MPFRFloat sinh(const MPFRFloat& value);                            \
qualifier MPFRFloat sqrt(const MPFRFloat& value);                            \
qualifier MPFRFloat tan(const MPFRFloat& value);                             \
qualifier MPFRFloat tanh(const MPFRFloat& value);                            \
qualifier int isfinite(const MPFRFloat& value);                              \
qualifier MPFRFloat copysign(const MPFRFloat& value1, const MPFRFloat& value2);\
template<typename char_t, typename traits_t>                                 \
qualifier                                                                    \
std::basic_ostream<char_t, traits_t>&                                        \
operator<<(std::basic_ostream<char_t, traits_t>& os, const MPFRFloat& f);    \
template<typename char_t, typename traits_t>                                 \
qualifier                                                                    \
std::basic_istream<char_t, traits_t>&                                        \
operator>>(std::basic_istream<char_t, traits_t>& os, MPFRFloat& f);

_FUNCTIONS_DECL(inline)

class MPFRFloat {
public:
  MPFRFloat()
  { mpfr_init2(_value, prec()); }
  MPFRFloat(const MPFRFloat& value)
  { mpfr_init2(_value, prec()); mpfr_set(_value, value._value, rm()); }
  MPFRFloat(const float& d)
  { mpfr_init2(_value, prec()); mpfr_set_d(_value, d, rm()); }
  MPFRFloat(const double& d)
  { mpfr_init2(_value, prec()); mpfr_set_d(_value, d, rm()); }
  MPFRFloat(const long double& ld)
  { mpfr_init2(_value, prec()); mpfr_set_ld(_value, ld, rm()); }
  MPFRFloat(const bool& si)
  { mpfr_init2(_value, prec()); mpfr_set_si(_value, si, rm()); }
  MPFRFloat(const signed char& si)
  { mpfr_init2(_value, prec()); mpfr_set_si(_value, si, rm()); }
  MPFRFloat(const short& si)
  { mpfr_init2(_value, prec()); mpfr_set_si(_value, si, rm()); }
  MPFRFloat(const int& si)
  { mpfr_init2(_value, prec()); mpfr_set_si(_value, si, rm()); }
  MPFRFloat(const long& si)
  { mpfr_init2(_value, prec()); mpfr_set_si(_value, si, rm()); }
  MPFRFloat(const unsigned char& ui)
  { mpfr_init2(_value, prec()); mpfr_set_ui(_value, ui, rm()); }
  MPFRFloat(const unsigned short& ui)
  { mpfr_init2(_value, prec()); mpfr_set_ui(_value, ui, rm()); }
  MPFRFloat(const unsigned int& ui)
  { mpfr_init2(_value, prec()); mpfr_set_ui(_value, ui, rm()); }
  MPFRFloat(const unsigned long& ui)
  { mpfr_init2(_value, prec()); mpfr_set_ui(_value, ui, rm()); }
  explicit MPFRFloat(const char* str)
  {
    mpfr_init2(_value, prec());
    mpfr_set_str(_value, str, std::strlen(str), rm());
  }
  explicit MPFRFloat(const std::string& str)
  {
    mpfr_init2(_value, prec());
    mpfr_set_str(_value, str.c_str(), str.size(), rm());
  }
  ~MPFRFloat()
  { mpfr_clear(_value); }

  MPFRFloat& operator=(const MPFRFloat& value)
  { mpfr_set(_value, value._value, rm()); return *this; }
  MPFRFloat& operator=(const bool& si)
  { mpfr_set_si(_value, si, rm()); return *this; }
  MPFRFloat& operator=(const signed char& si)
  { mpfr_set_si(_value, si, rm()); return *this; }
  MPFRFloat& operator=(const short& si)
  { mpfr_set_si(_value, si, rm()); return *this; }
  MPFRFloat& operator=(const int& si)
  { mpfr_set_si(_value, si, rm()); return *this; }
  MPFRFloat& operator=(const long& si)
  { mpfr_set_si(_value, si, rm()); return *this; }
  MPFRFloat& operator=(const unsigned char& ui)
  { mpfr_set_ui(_value, ui, rm()); return *this; }
  MPFRFloat& operator=(const unsigned short& ui)
  { mpfr_set_ui(_value, ui, rm()); return *this; }
  MPFRFloat& operator=(const unsigned int& ui)
  { mpfr_set_ui(_value, ui, rm()); return *this; }
  MPFRFloat& operator=(const unsigned long& ui)
  { mpfr_set_ui(_value, ui, rm()); return *this; }
  MPFRFloat& operator=(const float& d)
  { mpfr_set_d(_value, d, rm()); return *this; }
  MPFRFloat& operator=(const double& d)
  { mpfr_set_d(_value, d, rm()); return *this; }
  MPFRFloat& operator=(const long double& ld)
  { mpfr_set_ld(_value, ld, rm()); return *this; }
  MPFRFloat& operator=(const char* str)
  { mpfr_set_str(_value, str, std::strlen(str), rm()); return *this; }
  MPFRFloat& operator=(const std::string& str)
  { mpfr_set_str(_value, str.c_str(), str.size(), rm()); return *this; }

  operator bool() const
  { return mpfr_cmp_si(_value, 0); }
  operator signed char() const
  { return mpfr_get_si(_value, rm()); }
  operator short() const
  { return mpfr_get_si(_value, rm()); }
  operator int() const
  { return mpfr_get_si(_value, rm()); }
  operator long() const
  { return mpfr_get_si(_value, rm()); }
  operator unsigned char() const
  { return mpfr_get_ui(_value, rm()); }
  operator unsigned short() const
  { return mpfr_get_ui(_value, rm()); }
  operator unsigned long() const
  { return mpfr_get_ui(_value, rm()); }
  operator unsigned int() const
  { return mpfr_get_ui(_value, rm()); }
  operator float() const
  { return mpfr_get_d(_value, rm()); }
  operator double() const
  { return mpfr_get_d(_value, rm()); }
  operator long double() const
  { return mpfr_get_ld(_value, rm()); }

#define _INPLACE_OPERATOR(op, mop)                                  \
  MPFRFloat& operator op(const MPFRFloat& value)                    \
  { mpfr_##mop(_value, _value, value._value, rm()); return *this; } \
  MPFRFloat& operator op(const bool& value)                         \
  { mpfr_##mop##_si(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const signed char& value)                  \
  { mpfr_##mop##_si(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const signed short& value)                 \
  { mpfr_##mop##_si(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const int& value)                          \
  { mpfr_##mop##_si(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const long& value)                         \
  { mpfr_##mop##_si(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const unsigned char& value)                \
  { mpfr_##mop##_ui(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const unsigned short& value)               \
  { mpfr_##mop##_ui(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const unsigned int& value)                 \
  { mpfr_##mop##_ui(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const unsigned long& value)                \
  { mpfr_##mop##_ui(_value, _value, value, rm()); return *this; }   \
  MPFRFloat& operator op(const float& value)                        \
  { return operator op(MPFRFloat(value)); }                         \
  MPFRFloat& operator op(const double& value)                       \
  { return operator op(MPFRFloat(value)); }                         \
  MPFRFloat& operator op(const long double& value)                  \
  { return operator op(MPFRFloat(value)); }

  _INPLACE_OPERATOR(+=, add)
  _INPLACE_OPERATOR(-=, sub)
  _INPLACE_OPERATOR(*=, mul)
  _INPLACE_OPERATOR(/=, div)
#undef _INPLACE_OPERATOR

  MPFRFloat operator-() const
  {
    mpfr_t tmp;
    mpfr_init2(tmp, prec());
    mpfr_neg(tmp, _value, rm());
    return MPFRFloat(tmp);
  }

  static mpfr_rnd_t rm() { return GMP_RNDN; }
  static mpfr_prec_t prec() { return 128; }

  static MPFRFloat pi()
  {
    mpfr_t tmp;
    mpfr_init2(tmp, MPFRFloat::prec());
    mpfr_const_pi(tmp, rm());
    return MPFRFloat(tmp);
  }
  static MPFRFloat euler()
  {
    mpfr_t tmp;
    mpfr_init2(tmp, MPFRFloat::prec());
    mpfr_const_euler(tmp, rm());
    return MPFRFloat(tmp);
  }
  static MPFRFloat infinity()
  {
    mpfr_t tmp;
    mpfr_init2(tmp, MPFRFloat::prec());
    mpfr_set_inf(tmp, 1);
    return MPFRFloat(tmp);
  }
  static MPFRFloat NaN()
  {
    mpfr_t tmp;
    mpfr_init2(tmp, MPFRFloat::prec());
    mpfr_set_nan(tmp);
    return MPFRFloat(tmp);
  }

private:
  _BINARY_OPERATOR_DECL(friend,MPFRFloat,+)
  _BINARY_OPERATOR_DECL(friend,MPFRFloat,-)
  _BINARY_OPERATOR_DECL(friend,MPFRFloat,*)
  _BINARY_OPERATOR_DECL(friend,MPFRFloat,/)
  _BINARY_OPERATOR_DECL(friend,bool,==)
  _BINARY_OPERATOR_DECL(friend,bool,!=)
  _BINARY_OPERATOR_DECL(friend,bool,<)
  _BINARY_OPERATOR_DECL(friend,bool,<=)
  _BINARY_OPERATOR_DECL(friend,bool,>)
  _BINARY_OPERATOR_DECL(friend,bool,>=)
  _FUNCTIONS_DECL(friend)

  MPFRFloat(const mpfr_t value) { _value[0] = value[0]; }
  mpfr_t _value;
};

#undef _BINARY_OPERATOR_DECL
#undef _FUNCTIONS_DECL

#define _BINARY_OPERATOR_FPFP(op, mop)                                      \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const MPFRFloat& value2)               \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop(tmp, value1._value, value2._value, MPFRFloat::rm());           \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const float& value2)                   \
{                                                                           \
  mpfr_t tmp1;                                                              \
  mpfr_init_set_d(tmp1, value2, MPFRFloat::rm());                           \
  mpfr_t tmp2;                                                              \
  mpfr_init2(tmp2, MPFRFloat::prec());                                      \
  mpfr_##mop(tmp2, value1._value, tmp1, MPFRFloat::rm());                   \
  mpfr_clear(tmp1);                                                         \
  return MPFRFloat(tmp2);                                                   \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const float& value1, const MPFRFloat& value2)                   \
{                                                                           \
  mpfr_t tmp1;                                                              \
  mpfr_init_set_d(tmp1, value1, MPFRFloat::rm());                           \
  mpfr_t tmp2;                                                              \
  mpfr_init2(tmp2, MPFRFloat::prec());                                      \
  mpfr_##mop(tmp2, tmp1, value2._value, MPFRFloat::rm());                   \
  mpfr_clear(tmp1);                                                         \
  return MPFRFloat(tmp2);                                                   \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const double& value2)                  \
{                                                                           \
  mpfr_t tmp1;                                                              \
  mpfr_init_set_d(tmp1, value2, MPFRFloat::rm());                           \
  mpfr_t tmp2;                                                              \
  mpfr_init2(tmp2, MPFRFloat::prec());                                      \
  mpfr_##mop(tmp2, value1._value, tmp1, MPFRFloat::rm());                   \
  mpfr_clear(tmp1);                                                         \
  return MPFRFloat(tmp2);                                                   \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const double& value1, const MPFRFloat& value2)                  \
{                                                                           \
  mpfr_t tmp1;                                                              \
  mpfr_init_set_d(tmp1, value1, MPFRFloat::rm());                           \
  mpfr_t tmp2;                                                              \
  mpfr_init2(tmp2, MPFRFloat::prec());                                      \
  mpfr_##mop(tmp2, tmp1, value2._value, MPFRFloat::rm());                   \
  mpfr_clear(tmp1);                                                         \
  return MPFRFloat(tmp2);                                                   \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const long double& value2)             \
{                                                                           \
  mpfr_t tmp1;                                                              \
  mpfr_init_set_ld(tmp1, value2, MPFRFloat::rm());                          \
  mpfr_t tmp2;                                                              \
  mpfr_init2(tmp2, MPFRFloat::prec());                                      \
  mpfr_##mop(tmp2, value1._value, tmp1, MPFRFloat::rm());                   \
  mpfr_clear(tmp1);                                                         \
  return MPFRFloat(tmp2);                                                   \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const long double& value1, const MPFRFloat& value2)             \
{                                                                           \
  mpfr_t tmp1;                                                              \
  mpfr_init_set_ld(tmp1, value1, MPFRFloat::rm());                          \
  mpfr_t tmp2;                                                              \
  mpfr_init2(tmp2, MPFRFloat::prec());                                      \
  mpfr_##mop(tmp2, tmp1, value2._value, MPFRFloat::rm());                   \
  mpfr_clear(tmp1);                                                         \
  return MPFRFloat(tmp2);                                                   \
}


#define _BINARY_OPERATOR_FPI(op, mop)                                       \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const signed char& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_si(tmp, value1._value, value2, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const signed short& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_si(tmp, value1._value, value2, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const signed int& value2)              \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_si(tmp, value1._value, value2, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const signed long& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_si(tmp, value1._value, value2, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const unsigned char& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_ui(tmp, value1._value, value2, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const unsigned short& value2)          \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_ui(tmp, value1._value, value2, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const unsigned int& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_ui(tmp, value1._value, value2, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const MPFRFloat& value1, const unsigned long& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_ui(tmp, value1._value, value2, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}


#define _SYM_BINARY_OPERATOR_IFP(op, mop)                                   \
inline MPFRFloat                                                            \
operator op(const signed char& value1, const MPFRFloat& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_si(tmp, value2._value, value1, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const signed short& value1, const MPFRFloat& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_si(tmp, value2._value, value1, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const signed int& value1, const MPFRFloat& value2)              \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_si(tmp, value2._value, value1, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const signed long& value1, const MPFRFloat& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_si(tmp, value2._value, value1, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const unsigned char& value1, const MPFRFloat& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_ui(tmp, value2._value, value1, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const unsigned short& value1, const MPFRFloat& value2)          \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_ui(tmp, value2._value, value1, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const unsigned int& value1, const MPFRFloat& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_ui(tmp, value2._value, value1, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const unsigned long& value1, const MPFRFloat& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_##mop##_ui(tmp, value2._value, value1, MPFRFloat::rm());             \
  return MPFRFloat(tmp);                                                    \
}


#define _UNSYM_BINARY_OPERATOR_IFP(op, mop)                                 \
inline MPFRFloat                                                            \
operator op(const signed char& value1, const MPFRFloat& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_si_##mop(tmp, value1, value2._value, MPFRFloat::rm());               \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const signed short& value1, const MPFRFloat& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_si_##mop(tmp, value1, value2._value, MPFRFloat::rm());               \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const signed int& value1, const MPFRFloat& value2)              \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_si_##mop(tmp, value1, value2._value, MPFRFloat::rm());               \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const signed long& value1, const MPFRFloat& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_si_##mop(tmp, value1, value2._value, MPFRFloat::rm());               \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const unsigned char& value1, const MPFRFloat& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_ui_##mop(tmp, value1, value2._value, MPFRFloat::rm());               \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const unsigned short& value1, const MPFRFloat& value2)          \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_ui_##mop(tmp, value1, value2._value, MPFRFloat::rm());               \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const unsigned int& value1, const MPFRFloat& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_ui_##mop(tmp, value1, value2._value, MPFRFloat::rm());               \
  return MPFRFloat(tmp);                                                    \
}                                                                           \
inline MPFRFloat                                                            \
operator op(const unsigned long& value1, const MPFRFloat& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init2(tmp, MPFRFloat::prec());                                       \
  mpfr_ui_##mop(tmp, value1, value2._value, MPFRFloat::rm());               \
  return MPFRFloat(tmp);                                                    \
}

#define _SYM_BINARY_OPERATOR(op, mop) \
_BINARY_OPERATOR_FPFP(op, mop) \
_BINARY_OPERATOR_FPI(op, mop) \
_SYM_BINARY_OPERATOR_IFP(op, mop)

#define _UNSYM_BINARY_OPERATOR(op, mop) \
_BINARY_OPERATOR_FPFP(op, mop) \
_BINARY_OPERATOR_FPI(op, mop) \
_UNSYM_BINARY_OPERATOR_IFP(op, mop)

// now implement ...
_SYM_BINARY_OPERATOR(+, add)
_SYM_BINARY_OPERATOR(*, mul)
_UNSYM_BINARY_OPERATOR(-, sub)
_UNSYM_BINARY_OPERATOR(/, div)

#undef _BINARY_OPERATOR_FPFP
#undef _BINARY_OPERATOR_FPI
#undef _SYM_BINARY_OPERATOR_IFP
#undef _UNSYM_BINARY_OPERATOR_IFP
#undef _SYM_BINARY_OPERATOR
#undef _UNSYM_BINARY_OPERATOR

// Comparison
#define _COMPARISON_OPERATOR(op, mop)                                       \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const MPFRFloat& value2)               \
{                                                                           \
  return mpfr_##mop##_p(value1._value, value2._value);                      \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const float& value2)                   \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_d(tmp, value2, MPFRFloat::rm());                            \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const float& value1, const MPFRFloat& value2)                   \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_d(tmp, value1, MPFRFloat::rm());                            \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const double& value2)                  \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_d(tmp, value2, MPFRFloat::rm());                            \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const double& value1, const MPFRFloat& value2)                  \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_d(tmp, value1, MPFRFloat::rm());                            \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const long double& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ld(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const long double& value1, const MPFRFloat& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ld(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const signed char& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_si(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const signed short& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_si(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const signed int& value2)              \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_si(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const signed long& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_si(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const unsigned char& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ui(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const unsigned short& value2)          \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ui(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const unsigned int& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ui(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const MPFRFloat& value1, const unsigned long& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ui(tmp, value2, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(value1._value, tmp);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const signed char& value1, const MPFRFloat& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_si(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const signed short& value1, const MPFRFloat& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_si(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const signed int& value1, const MPFRFloat& value2)              \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_si(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const signed long& value1, const MPFRFloat& value2)             \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_si(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const unsigned char& value1, const MPFRFloat& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ui(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const unsigned short& value1, const MPFRFloat& value2)          \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ui(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const unsigned int& value1, const MPFRFloat& value2)            \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ui(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}                                                                           \
inline bool                                                                 \
operator op(const unsigned long& value1, const MPFRFloat& value2)           \
{                                                                           \
  mpfr_t tmp;                                                               \
  mpfr_init_set_ui(tmp, value1, MPFRFloat::rm());                           \
  int ret = mpfr_##mop##_p(tmp, value2._value);                             \
  mpfr_clear(tmp);                                                          \
  return ret;                                                               \
}

// now implement ...
_COMPARISON_OPERATOR(==, equal)
_COMPARISON_OPERATOR(!=, lessgreater)
_COMPARISON_OPERATOR(<, less)
_COMPARISON_OPERATOR(<=, lessequal)
_COMPARISON_OPERATOR(>, greater)
_COMPARISON_OPERATOR(>=, greaterequal)

#undef _COMPARISON_OPERATOR

inline
MPFRFloat
abs(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_abs(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
acos(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_acos(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
asin(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_asin(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
atan(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_atan(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
atan2(const MPFRFloat& value1, const MPFRFloat& value2)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_atan2(tmp, value1._value, value2._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
ceil(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_ceil(tmp, value._value);
  return MPFRFloat(tmp);
}

inline
MPFRFloat
cos(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_cos(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
cosh(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_cosh(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
exp(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_exp(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
fabs(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_abs(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
floor(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_floor(tmp, value._value);
  return MPFRFloat(tmp);
}

inline
MPFRFloat
fmod(const MPFRFloat& value1, const MPFRFloat& value2)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_remainder(tmp, value1._value, value2._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
frexp(const MPFRFloat& value1, int * value2)
{
  *value2 = mpfr_get_exp(value1._value);
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_set_exp(tmp, -*value2);
  mpfr_mul(tmp, tmp, value1._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
ldexp(const MPFRFloat& value1, int value2)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_set_si(tmp, 1, MPFRFloat::rm());
  mpfr_set_exp(tmp, value2);
  mpfr_mul(tmp, tmp, value1._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
log(const MPFRFloat& value)
{
  mpfr_t tmp; mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_log(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
log10(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_log10(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
pow(const MPFRFloat& value1, const MPFRFloat& value2)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_pow(tmp, value1._value, value2._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
sin(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_sin(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
sinh(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_sinh(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
sqrt(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_sqrt(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
tan(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_tan(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

inline
MPFRFloat
tanh(const MPFRFloat& value)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_tanh(tmp, value._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

// Not stdc++ functions but hopefully present ...
inline
int
isfinite(const MPFRFloat& value)
{
  return mpfr_number_p(value._value);
}

inline
MPFRFloat
copysign(const MPFRFloat& value1, const MPFRFloat& value2)
{
  mpfr_t tmp;
  mpfr_init2(tmp, MPFRFloat::prec());
  mpfr_copysign(tmp, value1._value, value2._value, MPFRFloat::rm());
  return MPFRFloat(tmp);
}

template<typename char_t, typename traits_t>
inline
std::basic_ostream<char_t, traits_t>&
operator<<(std::basic_ostream<char_t, traits_t>& os, const MPFRFloat& f)
{
  if (mpfr_sgn(f._value) < 0)
    os << os.widen('-');

  if (mpfr_nan_p(f._value))
    return os << os.widen('N') << os.widen('a') << os.widen('N');
  if (mpfr_inf_p(f._value))
    return os << os.widen('i') << os.widen('n') << os.widen('f');
  if (mpfr_zero_p(f._value))
    return os << os.widen('0');

  std::streamsize prec = os.precision();

  MPFRFloat tmp = abs(f);
  mp_exp_t exponent;
  char* s = mpfr_get_str(0, &exponent, 10 /*base*/,
                         prec, tmp._value, MPFRFloat::rm());

  if (0 < prec) {
    os << os.widen(s[0]);
    --prec;
    os << os.widen('.');
    for (unsigned i = 1; s[i] && 0 < prec; ++i) {
      os << os.widen(s[i]);
      --prec;
    }
    os << os.widen('e');
    os << exponent-1;
  }
  mpfr_free_str(s);
  return os;
}

template<typename char_t, typename traits_t>
inline
std::basic_istream<char_t, traits_t>&
operator>>(std::basic_istream<char_t, traits_t>& os, MPFRFloat& f)
{
  // FIXME
  long double tmp;
  os >> tmp;
  f = tmp;
  return os;
}

template<typename T>
struct Limits;

template<>
struct Limits<MPFRFloat> {
  static MPFRFloat epsilon()
  { return ldexp(MPFRFloat(1), 2 - MPFRFloat::prec()); }
  static MPFRFloat min()
  { return ldexp(MPFRFloat(1), mpfr_get_emin()); }
  static MPFRFloat safe_min()
  { return ldexp(MPFRFloat(1), MPFRFloat::prec() + mpfr_get_emin()); }
  static MPFRFloat max()
  { return ldexp(2*(MPFRFloat(1) - epsilon()), mpfr_get_emax()); }
  static MPFRFloat round_error()
  { return MPFRFloat::rm() == GMP_RNDN ? MPFRFloat(0.5) : MPFRFloat(1); }
  static MPFRFloat infinity() 
  { return MPFRFloat::infinity(); }
  static MPFRFloat quiet_NaN()
  { return MPFRFloat::NaN(); }
  static MPFRFloat signaling_NaN()
  { return MPFRFloat::NaN(); }
  static MPFRFloat denorm_min()
  { return min(); /*FIXME*/ }
};


template<typename T>
struct Constants;

template<>
struct Constants<MPFRFloat> {
  static MPFRFloat pi() { return MPFRFloat::pi(); }
  static MPFRFloat e() { return MPFRFloat::pi(); }
};

} // namespace OpenFDM

#endif
