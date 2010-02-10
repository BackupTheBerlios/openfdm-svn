/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinAlgProxy_H
#define OpenFDM_LinAlgProxy_H

namespace OpenFDM {

namespace LinAlg {

template<typename Impl, size_type m, size_type n>
class RangeExpr
  : public MatrixLValue<RangeExpr<Impl,m,n>,m,n> {
public:
  typedef Impl implementation_type;
  typedef typename implementation_type::value_type value_type;

  OpenFDM_FORCE_INLINE
  RangeExpr(Impl& ref, const Range& r1, const Range& r2)
    : ref_(ref), r1_(r1), r2_(r2)
  {}

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return r1_.last - r1_.first + 1; }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return r2_.last - r2_.first + 1; }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return ref_(r1_.first + i, r2_.first + j); }
  OpenFDM_FORCE_INLINE
  value_type& operator()(size_type i, size_type j)
  { return ref_(r1_.first + i, r2_.first + j); }

  OpenFDM_FORCE_INLINE
  bool resize(size_type i, size_type j)
  { return i == rows() && j == cols(); }

  OpenFDM_FORCE_INLINE
  RangeExpr& operator=(const RangeExpr& r)
  { assign(r); return *this; }
//   template<typename V, size_type m_, size_type n_>
//   OpenFDM_FORCE_INLINE
//   RangeExpr& operator=(const MatrixRValue<V,m_,n_>& A)
//   { assign(A); return *this; }

  OpenFDM_FORCE_INLINE
  RangeExpr& operator*=(value_type scalar)
  { scalarMultiplyMatrix(*this, scalar); return *this; }

private:
  implementation_type& ref_;
  Range r1_;
  Range r2_;
};


// // FIXME, this should not be required ...
// template<typename Impl1, typename Impl2>
// OpenFDM_FORCE_INLINE
// void swap(RangeExpr<Impl1>& m1, RangeExpr<Impl2>& m2)
// { swapMatrix(m1, m2);}

template<typename Impl, size_type m, size_type n>
class ConstRangeExpr
  : public MatrixRValue<ConstRangeExpr<Impl,m,n>,m,n> {
public:
  typedef Impl implementation_type;
  typedef typename implementation_type::value_type value_type;

  OpenFDM_FORCE_INLINE
  ConstRangeExpr(const Impl& ref, const Range& r1, const Range& r2)
    : ref_(ref), r1_(r1), r2_(r2)
  {}

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return r1_.last - r1_.first + 1; }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return r2_.last - r2_.first + 1; }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return ref_(r1_.first + i, r2_.first + j); }

private:
  const implementation_type& ref_;
  Range r1_;
  Range r2_;
};

template<typename Impl, size_type m, size_type n>
class MatrixPointerExpr
  : public MatrixLValue<MatrixPointerExpr<Impl,m,n>,m,n> {
public:
  typedef Impl implementation_type;
  typedef typename implementation_type::value_type value_type;

  OpenFDM_FORCE_INLINE
  MatrixPointerExpr(Impl& ref, const Range& r1, const Range& r2)
    : ptr_(ref.find(r1.first,r2.first)),
      rows_(1 + r1.last - r1.first),
      cols_(1 + r2.last - r2.first),
      ld_(ref.rows())
  {}

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return rows_; }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return cols_; }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return ptr_[i + j*ld_]; }
  OpenFDM_FORCE_INLINE
  value_type& operator()(size_type i, size_type j)
  { return ptr_[i + j*ld_]; }

  OpenFDM_FORCE_INLINE
  bool resize(size_type i, size_type j)
  { return i == rows() && j == cols(); }

  OpenFDM_FORCE_INLINE
  MatrixPointerExpr& operator=(const MatrixPointerExpr& r)
  { assign(r); return *this; }
  template<typename V, size_type m_, size_type n_>
  OpenFDM_FORCE_INLINE
  MatrixPointerExpr& operator=(const MatrixRValue<V,m_,n_>& A)
  { assign(A); return *this; }

  OpenFDM_FORCE_INLINE
  MatrixPointerExpr& operator*=(value_type scalar)
  { scalarMultiplyMatrix(*this, scalar); return *this; }

private:
  value_type* ptr_;
  size_type rows_;
  size_type cols_;
  size_type ld_;
};


template<typename Impl, size_type m, size_type n>
class ConstMatrixPointerExpr
  : public MatrixRValue<ConstMatrixPointerExpr<Impl,m,n>,m,n> {
public:
  typedef Impl implementation_type;
  typedef typename implementation_type::value_type value_type;

  OpenFDM_FORCE_INLINE
  ConstMatrixPointerExpr(const Impl& ref, const Range& r1, const Range& r2)
    : ptr_(ref.find(r1.first,r2.first)),
      rows_(1 + r1.last - r1.first),
      cols_(1 + r2.last - r2.first),
      ld_(ref.rows())
  {}

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return rows_; }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return cols_; }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return ptr_[i + j*ld_]; }

  OpenFDM_FORCE_INLINE
  bool resize(size_type i, size_type j)
  { return i == rows() && j == cols(); }

//   OpenFDM_FORCE_INLINE
//   RangeExpr& operator=(const RangeExpr& r)
//   { assign(r); return *this; }
//   template<typename V, size_type m_, size_type n_>
//   OpenFDM_FORCE_INLINE
//   RangeExpr& operator=(const MatrixRValue<V,m_,n_>& A)
//   { assign(A); return *this; }

private:
  const value_type* ptr_;
  size_type rows_;
  size_type cols_;
  size_type ld_;
};

} // namespace LinAlg

} // namespace OpenFDM

#endif
