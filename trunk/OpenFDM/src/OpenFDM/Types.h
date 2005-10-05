/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Types_H
#define OpenFDM_Types_H

namespace OpenFDM {

#if defined(__GNUC__) && (3 <= __GNUC__)
# define OpenFDM_Align_Double __attribute__((aligned(8)))
# define OpenFDM_Align_SSE2Double __attribute__((aligned(16)))
#else
# define OpenFDM_Align_Double
# define OpenFDM_Align_SSE2Double
#endif

//
// Definition of real_type.
// This one is used for *all* real numbers in OpenFDM.
//
#if defined(OPENFDM_USE_LONGDOUBLE)
typedef long double real_type;
#elif defined(OPENFDM_USE_DOUBLEDOUBLE)
// I have not tested that, just as an idea ...
# include <doubledouble.h>
typedef doubledouble real_type;
#else
typedef double OpenFDM_Align_Double real_type;
#endif

} // namespace OpenFDM

#endif
