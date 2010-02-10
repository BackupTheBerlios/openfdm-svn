/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Referenced_H
#define OpenFDM_Referenced_H

#include "OpenFDMConfig.h"
#include "Atomic.h"

namespace OpenFDM {

class Referenced {
public:
  Referenced(void) : _refcount(0u)
  {}
  /// Do not copy reference counts. Each new object has it's own counter
  Referenced(const Referenced&) : _refcount(0u)
  {}
  /// Do not copy reference counts. Each new object has it's own counter
  Referenced& operator=(const Referenced&)
  { return *this; }

  static void get(const Referenced* ref)
  { if (!ref) return; ++(ref->_refcount); }
  static void getFirst(const Referenced* ref)
  { get(ref); }
  static unsigned put(const Referenced* ref)
  { if (ref) return --(ref->_refcount); else return ~0u; }
  static void release(const Referenced* ref)
  { if (!ref) return; --(ref->_refcount); }
  static unsigned count(const Referenced* ref)
  { if (ref) return ref->_refcount; else return 0u; }
  static bool shared(const Referenced* ref)
  { if (ref) return 1u < ref->_refcount; else return false; }

  template<typename T>
  static void destroy(T* ref)
  { delete ref; }

private:
  mutable Atomic _refcount;
};

} // namespace OpenFDM

#endif
