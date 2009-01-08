/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_StateStream_H
#define OpenFDM_StateStream_H

#include "Vector.h"
#include "Matrix.h"

namespace OpenFDM {

class StateStream {
public:
  StateStream(unsigned nStates) : mOffset(0), mState(nStates)
  { }
  StateStream(const Vector& state) : mOffset(0), mState(state)
  { }
  ~StateStream(void)
  {
    OpenFDMAssert(mOffset == rows(mState));
  }

  void setState(const Vector& state)
  { mState = state; mOffset = 0; }
  const Vector& getState(void) const
  {
    OpenFDMAssert(mOffset == rows(mState));
    return mState;
  }

  /// Reset the offset to 0
  void reset(void)
  { mOffset = 0; }

  /// Debgging functions
  bool isAtEnd(void) const
  { return mOffset == rows(mState); }
  bool stillSpaceFor(unsigned nStates) const
  { return mOffset + nStates <= rows(mState); }

  OpenFDM_FORCE_INLINE
  void readSubState(real_type& value) const
  {
    OpenFDMAssert(mOffset + 1 <= rows(mState));
    value = mState(mOffset++);
  }
  template<typename Impl, LinAlg::size_type m, LinAlg::size_type n>
  OpenFDM_FORCE_INLINE
  void readSubState(LinAlg::MatrixLValue<Impl,m,n>& value) const
  {
    LinAlg::size_type r = rows(value);
    LinAlg::size_type c = cols(value);
    OpenFDMAssert(mOffset + r*c <= rows(mState));
    Impl& valueImpl = value.asImpl();
    for (LinAlg::size_type j = 0; j < c; ++j) 
      for (LinAlg::size_type i = 0; i < r; ++i)
        valueImpl(i, j) = mState(mOffset++);
  }
  template<typename Impl, LinAlg::size_type n>
  OpenFDM_FORCE_INLINE
  void readSubState(LinAlg::MatrixLValue<Impl,n,1>& value) const
  {
    LinAlg::size_type r = rows(value);
    OpenFDMAssert(mOffset + r <= rows(mState));
    Impl& valueImpl = value.asImpl();
    for (LinAlg::size_type i = 0; i < r; ++i)
      valueImpl(i) = mState(mOffset++);
  }

  OpenFDM_FORCE_INLINE
  void writeSubState(real_type value)
  {
    OpenFDMAssert(mOffset + 1 <= rows(mState));
    mState(mOffset++) = value;
  }
  template<typename Impl, LinAlg::size_type m, LinAlg::size_type n>
  OpenFDM_FORCE_INLINE
  void writeSubState(const LinAlg::MatrixRValue<Impl,m,n>& value)
  {
    LinAlg::size_type r = rows(value);
    LinAlg::size_type c = cols(value);
    OpenFDMAssert(mOffset + r*c <= rows(mState));
    const Impl& valueImpl = value.asImpl();
    for (LinAlg::size_type j = 0; j < c; ++j) 
      for (LinAlg::size_type i = 0; i < r; ++i)
        mState(mOffset++) = valueImpl(i, j);
  }
  template<typename Impl, LinAlg::size_type n>
  OpenFDM_FORCE_INLINE
  void writeSubState(const LinAlg::MatrixRValue<Impl,n,1>& value)
  {
    LinAlg::size_type r = rows(value);
    OpenFDMAssert(mOffset + r <= rows(mState));
    const Impl& valueImpl = value.asImpl();
    for (LinAlg::size_type i = 0; i < r; ++i)
      mState(mOffset++) = valueImpl(i);
  }

private:
  mutable unsigned mOffset;
  Vector mState;
};

} // namespace OpenFDM

#endif
