/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinAlgBase_H
#define OpenFDM_LinAlgBase_H

namespace OpenFDM {

namespace LinAlg {

// Forward decl.
template<typename Impl, size_type m, size_type n>
class MatrixRValue;

// Forward decl.
template<typename Impl, size_type m, size_type n>
class MatrixLValue;

template<typename Impl>
OpenFDM_FORCE_INLINE
void clearMatrix(Impl&);

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void assignMatrix(Impl1&, const Impl2&);

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void directAssignMatrix(Impl1&, const Impl2&);

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void plusAssignMatrix(Impl1&, const Impl2&);

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void minusAssignMatrix(Impl1&, const Impl2&);

template<typename Impl>
OpenFDM_FORCE_INLINE
void scalarMultiplyMatrix(Impl&, typename Impl::value_type);

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void swapMatrix(Impl1&, Impl2&);

// Helper class to make handling of that stuff more safe.
class NonAssignable {
public:
  OpenFDM_FORCE_INLINE
  NonAssignable(void)
  { }
  OpenFDM_FORCE_INLINE
  ~NonAssignable(void)
  { }

private:
  NonAssignable& operator=(const NonAssignable&);
};

// Base class for everything which behaves like a matrix.
// We expect from a matrix implementation to have the following functions.
//   size_type rows(void) const
//   size_type cols(void) const
//   value_type operator()(size_type i, size_type j) const
// and additionally for expressions capable as lvalues:
//   bool resize(size_type i, size_type j)
//   value_type& operator()(size_type i, size_type j)
// 
template<typename Impl, size_type m, size_type n>
class MatrixRValue
  : private NonAssignable {
public:
  // Make the original implementation type available.
  typedef Impl implementation_type;

  OpenFDM_FORCE_INLINE
  MatrixRValue(void)
  {}
  OpenFDM_FORCE_INLINE
  ~MatrixRValue(void)
  {}

  // For the Barton - Nackman trick.
  OpenFDM_FORCE_INLINE
  Impl& asImpl(void)
  { return *static_cast<Impl*>(this); }
  OpenFDM_FORCE_INLINE
  const Impl& asImpl(void) const
  { return *static_cast<const Impl*>(this); }
};

template<typename Impl, size_type rows_, size_type cols_>
class MatrixLValue
  : public MatrixRValue<Impl,rows_,cols_> {
public:
  // Make the original implementation type available.
  typedef Impl implementation_type;

  OpenFDM_FORCE_INLINE
  MatrixLValue(void)
  {}
  OpenFDM_FORCE_INLINE
  ~MatrixLValue(void)
  {}

  // Some common methods implemented here.
  // FIXME: might move into those ones being an lvalue??
  OpenFDM_FORCE_INLINE
  void clear(void)
  { clearMatrix(asImpl()); }
  template<typename V, size_type m, size_type n>
  OpenFDM_FORCE_INLINE
  void assign(const MatrixRValue<V,m,n>& A)
  {
    // Note that the assignment is first here...
#ifdef USE_EXPRESSIONS
    assignMatrix(asImpl(), A.asImpl());
#else
    directAssignMatrix(asImpl(), A.asImpl());
#endif
    SizeCheck<rows_,m>::Equal(asImpl().rows(), A.asImpl().rows());
    SizeCheck<cols_,n>::Equal(asImpl().cols(), A.asImpl().cols());
  }
  template<typename V, size_type m, size_type n>
  OpenFDM_FORCE_INLINE
  void directAssign(const MatrixRValue<V,m,n>& A)
  {
    // Note that the assignment is first here...
    directAssignMatrix(asImpl(), A.asImpl());
    SizeCheck<rows_,m>::Equal(asImpl().rows(), A.asImpl().rows());
    SizeCheck<cols_,n>::Equal(asImpl().cols(), A.asImpl().cols());
  }
  template<typename V, size_type m, size_type n>
  OpenFDM_FORCE_INLINE
  void plusAssign(const MatrixRValue<V,m,n>& A)
  {
    SizeCheck<rows_,m>::Equal(asImpl().rows(), A.asImpl().rows());
    SizeCheck<cols_,n>::Equal(asImpl().cols(), A.asImpl().cols());
    plusAssignMatrix(asImpl(), A.asImpl());
  }
  template<typename V, size_type m, size_type n>
  OpenFDM_FORCE_INLINE
  void minusAssign(const MatrixRValue<V,m,n>& A)
  {
    SizeCheck<rows_,m>::Equal(asImpl().rows(), A.asImpl().rows());
    SizeCheck<cols_,n>::Equal(asImpl().cols(), A.asImpl().cols());
    minusAssignMatrix(asImpl(), A.asImpl());
  }

  template<typename V, size_type m, size_type n>
  OpenFDM_FORCE_INLINE
  MatrixLValue& operator=(const MatrixRValue<V,m,n>& A)
  { assign(A); return *this; }

  template<typename V, size_type m, size_type n>
  OpenFDM_FORCE_INLINE
  MatrixLValue& operator+=(const MatrixRValue<V,m,n>& A)
  { plusAssign(A); return *this; }

  template<typename V, size_type m, size_type n>
  OpenFDM_FORCE_INLINE
  MatrixLValue& operator-=(const MatrixRValue<V,m,n>& A)
  { minusAssign(A); return *this; }

  // For the Barton - Nackman trick.
  using MatrixRValue<Impl,rows_,cols_>::asImpl;
};

template<typename Impl>
OpenFDM_FORCE_INLINE
void clearMatrix(Impl& m)
{
  typedef typename Impl::value_type value_type;

  size_type rows = m.rows();
  size_type cols = m.cols();
  for (size_type j = 0; j < cols; ++j) {
    for (size_type i = 0; i < rows; ++i) {
      m(i, j) = static_cast<value_type>(0);
    }
  }
}

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void assignMatrix(Impl1& m1, const Impl2& m2)
{
  size_type rows = m2.rows();
  size_type cols = m2.cols();

  Impl1 tmp(rows, cols);
  for (size_type j = 0; j < cols; ++j) {
    for (size_type i = 0; i < rows; ++i) {
      tmp(i, j) = m2(i, j);
    }
  }

  bool resized = m1.resize(rows, cols);
  OpenFDMLinAlgAssert(resized);

  for (size_type j = 0; j < cols; ++j) {
    for (size_type i = 0; i < rows; ++i) {
      m1(i, j) = tmp(i, j);
    }
  }
}

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void directAssignMatrix(Impl1& m1, const Impl2& m2)
{
  size_type rows = m2.rows();
  size_type cols = m2.cols();

  bool resized = m1.resize(rows, cols);
  OpenFDMLinAlgAssert(resized);

  for (size_type j = 0; j < cols; ++j) {
    for (size_type i = 0; i < rows; ++i) {
      m1(i, j) = m2(i, j);
    }
  }
}

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void plusAssignMatrix(Impl1& m1, const Impl2& m2)
{
  size_type rows = m2.rows();
  size_type cols = m2.cols();
  for (size_type j = 0; j < cols; ++j) {
    for (size_type i = 0; i < rows; ++i) {
      m1(i, j) += m2(i, j);
    }
  }
}

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void minusAssignMatrix(Impl1& m1, const Impl2& m2)
{
  size_type rows = m2.rows();
  size_type cols = m2.cols();
  for (size_type j = 0; j < cols; ++j) {
    for (size_type i = 0; i < rows; ++i) {
      m1(i, j) -= m2(i, j);
    }
  }
}

template<typename Impl>
OpenFDM_FORCE_INLINE
void scalarMultiplyMatrix(Impl& m, typename Impl::value_type scalar)
{
  size_type rows = m.rows();
  size_type cols = m.cols();
  for (size_type j = 0; j < cols; ++j) {
    for (size_type i = 0; i < rows; ++i) {
      m(i, j) *= scalar;
    }
  }
}

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
void swapMatrix(Impl1& m1, Impl2& m2)
{
  typedef typename Impl1::value_type value_type1;
  typedef typename Impl2::value_type value_type2;

  size_type rows1 = m1.rows();
  size_type cols1 = m1.cols();
  OpenFDMLinAlgAssert(rows1 == m2.rows());
  OpenFDMLinAlgAssert(cols1 == m2.cols());
  for (size_type j = 0; j < cols1; ++j) {
    for (size_type i = 0; i < rows1; ++i) {
      value_type2 tmp = static_cast<value_type2>(m1(i, j));
      m1(i, j) = static_cast<value_type1>(m2(i, j));
      m2(i, j) = tmp;
    }
  }
}

template<size_type m1, size_type n1, typename Impl1, size_type m2, size_type n2, typename Impl2>
OpenFDM_FORCE_INLINE
void swap(MatrixLValue<Impl1,m1,n1>& A1, MatrixLValue<Impl2,m2,n2>& A2)
{
  SizeCheck<m1,m2>::Equal(A1.asImpl().rows(), A2.asImpl().rows());
  SizeCheck<n1,n2>::Equal(A1.asImpl().cols(), A2.asImpl().cols());
  swapMatrix(A1.asImpl(), A2.asImpl());
}

} // namespace LinAlg

} // namespace OpenFDM

#endif
