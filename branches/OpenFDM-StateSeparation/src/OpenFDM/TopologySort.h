/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TopologySort_H
#define OpenFDM_TopologySort_H

#include <list>

namespace OpenFDM {

/// Topological sorting.
/// On success, the sourceList is empty and the sortedList contains the sorted
/// entries.
/// On failure, the sourceList contains those entries that could be sorted, and
/// the sourceList still contains those that have cycles.
template<typename T, typename D>
static inline void
tsort(std::list<T>& sortedList, std::list<T>& sourceList, const D& dependency)
{
  /// The algorithm is simple.
  /// Just push all those items from the sourceList into the sortedList
  /// that do not depend on any item still in the sourceList.
  /// Items that do depend on anything still in the source list are
  /// moved to the tail of the sourceList.
  /// Do that until sourceList is empty or until we do no longer find any
  /// independent item.
  typename std::list<T>::size_type size = sourceList.size();
  typename std::list<T>::size_type tryCount = 0;
  while (!sourceList.empty() && tryCount < size) {
    
    // Check if the front item is independent of sourceList
    typename std::list<T>::iterator i = sourceList.begin();
    for (; i != sourceList.end(); ++i) {
      if (dependency(sourceList.front(), *i))
        break;
    }
    // If we came here with i == end, the front item is independent so
    // schedule it now
    if (i == sourceList.end()) {
      sortedList.splice(sortedList.end(), sourceList, sourceList.begin());
      --size;
      tryCount = 0;
    }
    // Else defer that item
    else {
      sourceList.splice(sourceList.end(), sourceList, sourceList.begin());
      ++tryCount;
    }
  }
}

} // namespace OpenFDM

#endif
