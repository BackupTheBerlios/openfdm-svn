/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Limits_H
#define OpenFDM_Limits_H

#include <limits>

#include "Types.h"

namespace OpenFDM {

/// Class containing numeric limits for the numeric types in use by OpenFDM.
/// That is actually the same than std::numeric_limits for the usual types.
/// For custom types than dd or qd we need to specialize that class further.
/// So this is one of the three places where modifications for special
/// real types must be made (@see Types.h, @see Math.h).
template<typename T>
struct Limits : public std::numeric_limits<T> {
   /// smalles value so that 1/safe_min is still finite
   static T safe_min()
   { return std::max((T(1) + std::numeric_limits<T>::epsilon())/std::numeric_limits<T>::max(), std::numeric_limits<T>::min()); }
   /// minimum value this type can represent
   static T min_value()
   { return std::min(std::numeric_limits<T>::min(), -std::numeric_limits<T>::max()); }
   /// maximum value this type can represent
   static T max_value()
   { return std::numeric_limits<T>::max(); }
};

} // namespace OpenFDM

#endif
