/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Referenced_H
#define OpenFDM_Referenced_H

namespace OpenFDM {

class Referenced {
public:
  Referenced(void) : _refcount(0u) {}

  static unsigned get(const Referenced* ref)
  { if (ref) return ++(ref->_refcount); else return ~0u; }
  static unsigned put(const Referenced* ref)
  { if (ref) return --(ref->_refcount); else return ~0u; }
  static unsigned count(const Referenced* ref)
  { if (ref) return ref->_refcount; else return ~0u; }
  static bool shared(const Referenced* ref)
  { if (ref) return 1u < ref->_refcount; else return false; }

private:
  mutable unsigned _refcount;
};

} // namespace OpenFDM

#endif
