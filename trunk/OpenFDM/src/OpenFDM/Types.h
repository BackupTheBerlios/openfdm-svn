/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Types_H
#define OpenFDM_Types_H

#include "OpenFDMConfig.h"

#if defined(OPENFDM_USE_MPFR)
#include "MPFRFloat.h"
#endif

namespace OpenFDM {

#if defined(__GNUC__) && (3 <= __GNUC__)
# define OpenFDM_Align_Double __attribute__((aligned(8)))
# define OpenFDM_Align_SSE2Double __attribute__((aligned(16)))
#elif defined(__MSVC__)
# define OpenFDM_Align_Double __declspec(align(8))
# define OpenFDM_Align_SSE2Double __declspec(align(16))
#else
# define OpenFDM_Align_Double
# define OpenFDM_Align_SSE2Double
#endif

//
// Be sure to inline 
#if defined(__GNUC__) && (4 <= __GNUC__) && defined(__OPTIMIZE__)
# define OpenFDM_FORCE_INLINE inline __attribute__((always_inline))
#elif defined(__MSVC__)
# define OpenFDM_FORCE_INLINE __forceinline
#else
# define OpenFDM_FORCE_INLINE inline
#endif

//
// Definition of real_type.
// This one is used for *all* real numbers in OpenFDM.
//
#if defined(OPENFDM_USE_MPFR)
typedef MPFRFloat real_type;
#elif defined(OPENFDM_USE_LONGDOUBLE)
typedef long double real_type;
#else
typedef double real_type;
#endif

} // namespace OpenFDM

#endif
