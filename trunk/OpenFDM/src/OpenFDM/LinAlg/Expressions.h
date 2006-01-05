/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinAlgExpressions_H
#define OpenFDM_LinAlgExpressions_H

namespace OpenFDM {

namespace LinAlg {

OpenFDM_FORCE_INLINE
real_type min(real_type a, real_type b)
{ return a < b ? a : b; }

OpenFDM_FORCE_INLINE
real_type max(real_type a, real_type b)
{ return a < b ? b : a; }

#ifdef USE_EXPRESSIONS

template<typename Impl, size_type m, size_type n>
class TransposeExpr
  : public MatrixRValue<TransposeExpr<Impl,m,n>,m,n> {
public:
  typedef Impl implementation_type;
  typedef typename implementation_type::value_type value_type;

  OpenFDM_FORCE_INLINE
  TransposeExpr(const implementation_type& ref)
    : ref_(ref)
  {}
  OpenFDM_FORCE_INLINE
  ~TransposeExpr(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return ref_.cols(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return ref_.rows(); }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return ref_(j, i); }

private:
  const implementation_type& ref_;
};

// Returns a matrix expression representing a transposed matrix.
template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
TransposeExpr<Impl,m,n>
trans(const MatrixRValue<Impl,n,m>& A)
{ return TransposeExpr<Impl,m,n>(A.asImpl()); }

template<typename Impl, size_type m, size_type n>
class MatrixUnaryMinusExpr
  : public MatrixRValue<MatrixUnaryMinusExpr<Impl,m,n>,m,n> {
public:
  typedef Impl implementation_type;
  typedef typename implementation_type::value_type value_type;

  OpenFDM_FORCE_INLINE
  MatrixUnaryMinusExpr(const Impl& ref)
    : ref_(ref)
  {}
  OpenFDM_FORCE_INLINE
  ~MatrixUnaryMinusExpr(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return ref_.rows(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return ref_.cols(); }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return -ref_(i, j); }

private:
  const implementation_type& ref_;
};

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
MatrixUnaryMinusExpr<Impl,m,n>
operator-(const MatrixRValue<Impl,m,n>& A)
{ return MatrixUnaryMinusExpr<Impl,m,n>(A.asImpl()); }

template<typename Impl, size_type m, size_type n>
class MatrixScalarMultiplyExpr
  : public MatrixRValue<MatrixScalarMultiplyExpr<Impl,m,n>,m,n> {
public:
  typedef Impl implementation_type;
  typedef typename implementation_type::value_type value_type;

  OpenFDM_FORCE_INLINE
  MatrixScalarMultiplyExpr(const Impl& ref, value_type scalar)
    : ref_(ref), scalar_(scalar)
  {}
  OpenFDM_FORCE_INLINE
  ~MatrixScalarMultiplyExpr(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return ref_.rows(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return ref_.cols(); }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return scalar_*ref_(i, j); }

private:
  const implementation_type& ref_;
  value_type scalar_;
};

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
MatrixScalarMultiplyExpr<Impl,m,n>
operator*(typename Impl::value_type scalar, const MatrixRValue<Impl,m,n>& A)
{ return MatrixScalarMultiplyExpr<Impl,m,n>(A.asImpl(), scalar); }

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
MatrixScalarMultiplyExpr<Impl,m,n>
operator*(const MatrixRValue<Impl,m,n>& A, typename Impl::value_type scalar)
{ return MatrixScalarMultiplyExpr<Impl,m,n>(A.asImpl(), scalar); }

template<typename Impl1, typename Impl2, size_type m, size_type n>
class MatrixBinaryPlusExpr
  : public MatrixRValue<MatrixBinaryPlusExpr<Impl1,Impl2,m,n>,m,n> {
public:
  typedef Impl1 implementation1_type;
  typedef typename implementation1_type::value_type value1_type;
  typedef Impl2 implementation2_type;
  typedef typename implementation2_type::value_type value2_type;

  // FIXME: is usually correct,but ...
  typedef value1_type value_type;

  OpenFDM_FORCE_INLINE
  MatrixBinaryPlusExpr(const Impl1& op1, const Impl2& op2)
    : op1_(op1), op2_(op2)
  {}
  OpenFDM_FORCE_INLINE
  ~MatrixBinaryPlusExpr(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return op1_.rows(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return op1_.cols(); }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return op1_(i, j) + op2_(i, j); }

private:
  const implementation1_type& op1_;
  const implementation2_type& op2_;
};

template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
MatrixBinaryPlusExpr<Impl1,Impl2,m1,n1>
operator+(const MatrixRValue<Impl1,m1,n1>& A1,
          const MatrixRValue<Impl2,m2,n2>& A2)
{
  SizeCheck<m1,m2>::Equal(A1.asImpl().rows(), A2.asImpl().rows());
  SizeCheck<n1,n2>::Equal(A1.asImpl().cols(), A2.asImpl().cols());
  return MatrixBinaryPlusExpr<Impl1,Impl2,m1,n1>(A1.asImpl(), A2.asImpl());
}

template<typename Impl1, typename Impl2, size_type m, size_type n>
class MatrixBinaryMinusExpr
  : public MatrixRValue<MatrixBinaryMinusExpr<Impl1,Impl2,m,n>,m,n> {
public:
  typedef Impl1 implementation1_type;
  typedef typename implementation1_type::value_type value1_type;
  typedef Impl2 implementation2_type;
  typedef typename implementation2_type::value_type value2_type;

  // FIXME: is usually correct,but ...
  typedef value1_type value_type;

  OpenFDM_FORCE_INLINE
  MatrixBinaryMinusExpr(const Impl1& op1, const Impl2& op2)
    : op1_(op1), op2_(op2)
  {}
  OpenFDM_FORCE_INLINE
  ~MatrixBinaryMinusExpr(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return op1_.rows(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return op1_.cols(); }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return op1_(i, j) - op2_(i, j); }

private:
  const implementation1_type& op1_;
  const implementation2_type& op2_;
};

template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
MatrixBinaryMinusExpr<Impl1,Impl2,m1,n1>
operator-(const MatrixRValue<Impl1,m1,n1>& A1,
          const MatrixRValue<Impl2,m2,n2>& A2)
{
  SizeCheck<m1,m2>::Equal(A1.asImpl().rows(), A2.asImpl().rows());
  SizeCheck<n1,n2>::Equal(A1.asImpl().cols(), A2.asImpl().cols());
  return MatrixBinaryMinusExpr<Impl1,Impl2,m1,n1>(A1.asImpl(), A2.asImpl());
}

template<typename Impl1, typename Impl2, size_type m, size_type n>
class MatrixBinaryProductExpr
  : public MatrixRValue<MatrixBinaryProductExpr<Impl1,Impl2,m,n>,m,n> {
public:
  typedef Impl1 implementation1_type;
  typedef typename implementation1_type::value_type value1_type;
  typedef Impl2 implementation2_type;
  typedef typename implementation2_type::value_type value2_type;

  // FIXME: is usually correct,but ...
  typedef value1_type value_type;

  OpenFDM_FORCE_INLINE
  MatrixBinaryProductExpr(const Impl1& op1, const Impl2& op2)
    : op1_(op1), op2_(op2)
  { }
  OpenFDM_FORCE_INLINE
  ~MatrixBinaryProductExpr(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return op1_.rows(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return op2_.cols(); }
  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  {
    value_type ret = 0;
    size_type cols = op1_.cols();
    size_type k;
    for (k = 1; k <= cols; ++k)
      ret += op1_(i, k) * op2_(k, j);
    return ret;
  }

private:
  const implementation1_type& op1_;
  const implementation2_type& op2_;
};

template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
MatrixBinaryProductExpr<Impl1,Impl2,m1,n2>
operator*(const MatrixRValue<Impl1,m1,n1>& A1,
          const MatrixRValue<Impl2,m2,n2>& A2)
{
  SizeCheck<n1,m2>::Equal(A1.asImpl().cols(), A2.asImpl().rows());
  return MatrixBinaryProductExpr<Impl1,Impl2,m1,n2>(A1.asImpl(), A2.asImpl());
}

#else

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
Matrix<typename Impl::value_type,m,n>
trans(const MatrixRValue<Impl,n,m>& A)
{
  const Impl& Ai = A.asImpl();

  size_type rows = Ai.rows();
  size_type cols = Ai.cols();

  Matrix<typename Impl::value_type,m,n> ret(cols, rows);
  size_type i, j;
  for (j = 1; j <= cols; ++j)
    for (i = 1; i <= rows; ++i)
      ret(j,i) = Ai(i,j);

  return ret;
}

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
Matrix<typename Impl::value_type,m,n>
operator-(const MatrixRValue<Impl,m,n>& A)
{
  const Impl& Ai = A.asImpl();

  size_type rows = Ai.rows();
  size_type cols = Ai.cols();

  Matrix<typename Impl::value_type,m,n> ret(rows, cols);
  size_type i, j;
  for (j = 1; j <= cols; ++j)
    for (i = 1; i <= rows; ++i)
      ret(i,j) = -Ai(i,j);

  return ret;
}

template<typename Impl, size_type m>
OpenFDM_FORCE_INLINE
Vector<typename Impl::value_type,m>
operator-(const MatrixRValue<Impl,m,1>& A)
{
  const Impl& Ai = A.asImpl();

  size_type rows = Ai.rows();

  Vector<typename Impl::value_type,m> ret(rows, 1);
  size_type i;
  for (i = 1; i <= rows; ++i)
    ret(i,1) = -Ai(i,1);

  return ret;
}

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
Matrix<typename Impl::value_type,m,n>
operator*(typename Impl::value_type scalar, const MatrixRValue<Impl,m,n>& A)
{
  const Impl& Ai = A.asImpl();

  size_type rows = Ai.rows();
  size_type cols = Ai.cols();

  Matrix<typename Impl::value_type,m,n> ret(rows, cols);
  size_type i, j;
  for (j = 1; j <= cols; ++j)
    for (i = 1; i <= rows; ++i)
      ret(i,j) = scalar*Ai(i,j);

  return ret;
}

template<typename Impl, size_type m>
OpenFDM_FORCE_INLINE
Vector<typename Impl::value_type,m>
operator*(typename Impl::value_type scalar, const MatrixRValue<Impl,m,1>& A)
{
  const Impl& Ai = A.asImpl();

  size_type rows = Ai.rows();

  Vector<typename Impl::value_type,m> ret(rows, 1);
  size_type i;
  for (i = 1; i <= rows; ++i)
    ret(i,1) = scalar*Ai(i,1);

  return ret;
}

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
Matrix<typename Impl::value_type,m,n>
operator*(const MatrixRValue<Impl,m,n>& A, typename Impl::value_type scalar)
{
  const Impl& Ai = A.asImpl();

  size_type rows = Ai.rows();
  size_type cols = Ai.cols();

  Matrix<typename Impl::value_type,m,n> ret(rows, cols);
  size_type i, j;
  for (j = 1; j <= cols; ++j)
    for (i = 1; i <= rows; ++i)
      ret(i,j) = scalar*Ai(i,j);

  return ret;
}

template<typename Impl, size_type m>
OpenFDM_FORCE_INLINE
Vector<typename Impl::value_type,m>
operator*(const MatrixRValue<Impl,m,1>& A, typename Impl::value_type scalar)
{
  const Impl& Ai = A.asImpl();

  size_type rows = Ai.rows();

  Vector<typename Impl::value_type,m> ret(rows, 1);
  size_type i;
  for (i = 1; i <= rows; ++i)
    ret(i,1) = scalar*Ai(i,1);

  return ret;
}

template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,m1,n1>
operator+(const MatrixRValue<Impl1,m1,n1>& A1,
          const MatrixRValue<Impl2,m2,n2>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  size_type rows = A1i.rows();
  size_type cols = A1i.cols();
  SizeCheck<m1,m2>::Equal(rows, A2i.rows());
  SizeCheck<n1,n2>::Equal(cols, A2i.cols());

  Matrix<typename Impl1::value_type,m1,n1> ret(rows, cols);
  size_type i, j;
  for (j = 1; j <= cols; ++j)
    for (i = 1; i <= rows; ++i)
      ret(i, j) = A1i(i, j) + A2i(i, j);

  return ret;
}

template<typename Impl1, size_type m1,
         typename Impl2, size_type m2>
OpenFDM_FORCE_INLINE
Vector<typename Impl1::value_type,m1>
operator+(const MatrixRValue<Impl1,m1,1>& A1,
          const MatrixRValue<Impl2,m2,1>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  size_type rows = A1i.rows();
  SizeCheck<m1,m2>::Equal(rows, A2i.rows());

  Vector<typename Impl1::value_type,m1> ret(rows, 1);
  size_type i;
  for (i = 1; i <= rows; ++i)
    ret(i, 1) = A1i(i, 1) + A2i(i, 1);

  return ret;
}

template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,m1,n1>
operator-(const MatrixRValue<Impl1,m1,n1>& A1,
          const MatrixRValue<Impl2,m2,n2>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  size_type rows = A1i.rows();
  size_type cols = A1i.cols();
  SizeCheck<m1,m2>::Equal(rows, A2i.rows());
  SizeCheck<n1,n2>::Equal(cols, A2i.cols());

  Matrix<typename Impl1::value_type,m1,n1> ret(rows, cols);
  size_type i, j;
  for (j = 1; j <= cols; ++j)
    for (i = 1; i <= rows; ++i)
      ret(i, j) = A1i(i, j) - A2i(i, j);

  return ret;
}

template<typename Impl1, size_type m1,
         typename Impl2, size_type m2>
OpenFDM_FORCE_INLINE
Vector<typename Impl1::value_type,m1>
operator-(const MatrixRValue<Impl1,m1,1>& A1,
          const MatrixRValue<Impl2,m2,1>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  size_type rows = A1i.rows();
  SizeCheck<m1,m2>::Equal(rows, A2i.rows());

  Vector<typename Impl1::value_type,m1> ret(rows, 1);
  size_type i;
  for (i = 1; i <= rows; ++i)
    ret(i, 1) = A1i(i, 1) - A2i(i, 1);

  return ret;
}

template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,m1,n2>
operator*(const MatrixRValue<Impl1,m1,n1>& A1,
          const MatrixRValue<Impl2,m2,n2>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  size_type m = A1i.rows();
  size_type k = A1i.cols();
  size_type n = A2i.cols();

  SizeCheck<n1,m2>::Equal(A1i.cols(), A2i.rows());

  typedef typename Impl1::value_type value_type;

  Matrix<value_type,m1,n2> ret(m, n);
  size_type j;
  for (j = 1; j <= n; ++j) {
    size_type i;
    value_type t = A2i(1, j);
    for (i = 1; i <= m; ++i) {
      ret(i, j) = t*A1i(i, 1);
    }
    size_type l;
    for (l = 2; l <= k; ++l) {
      t = A2i(l, j);
      for (i = 1; i <= m; ++i) {
        ret(i, j) += t*A1i(i, l);
      }
    }
  }
  return ret;
}

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,3,3>
operator*(const MatrixRValue<Impl1,3,3>& A1,
          const MatrixRValue<Impl2,3,3>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  typedef typename Impl1::value_type value_type;
  Matrix<typename Impl1::value_type,3,3> ret;
  ret(1,1) = A1i(1,1)*A2i(1,1) + A1i(1,2)*A2i(2,1) + A1i(1,3)*A2i(3,1);
  ret(2,1) = A1i(2,1)*A2i(1,1) + A1i(2,2)*A2i(2,1) + A1i(2,3)*A2i(3,1);
  ret(3,1) = A1i(3,1)*A2i(1,1) + A1i(3,2)*A2i(2,1) + A1i(3,3)*A2i(3,1);
  ret(1,2) = A1i(1,1)*A2i(1,2) + A1i(1,2)*A2i(2,2) + A1i(1,3)*A2i(3,2);
  ret(2,2) = A1i(2,1)*A2i(1,2) + A1i(2,2)*A2i(2,2) + A1i(2,3)*A2i(3,2);
  ret(3,2) = A1i(3,1)*A2i(1,2) + A1i(3,2)*A2i(2,2) + A1i(3,3)*A2i(3,2);
  ret(1,3) = A1i(1,1)*A2i(1,3) + A1i(1,2)*A2i(2,3) + A1i(1,3)*A2i(3,3);
  ret(2,3) = A1i(2,1)*A2i(1,3) + A1i(2,2)*A2i(2,3) + A1i(2,3)*A2i(3,3);
  ret(3,3) = A1i(3,1)*A2i(1,3) + A1i(3,2)*A2i(2,3) + A1i(3,3)*A2i(3,3);
  return ret;
}

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
Vector<typename Impl1::value_type,3>
operator*(const MatrixRValue<Impl1,3,3>& A1,
          const MatrixRValue<Impl2,3,1>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  typedef typename Impl1::value_type value_type;
  Vector<typename Impl1::value_type,3> ret;
  ret(1) = A1i(1,1)*A2i(1,1) + A1i(1,2)*A2i(2,1) + A1i(1,3)*A2i(3,1);
  ret(2) = A1i(2,1)*A2i(1,1) + A1i(2,2)*A2i(2,1) + A1i(2,3)*A2i(3,1);
  ret(3) = A1i(3,1)*A2i(1,1) + A1i(3,2)*A2i(2,1) + A1i(3,3)*A2i(3,1);
  return ret;
}
using LinAlg::Zeros;

template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
Vector<typename Impl1::value_type,6>
operator*(const MatrixRValue<Impl1,6,6>& A1,
          const MatrixRValue<Impl2,6,1>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  typedef typename Impl1::value_type value_type;
  Vector<typename Impl1::value_type,6> ret;
  ret(1) = A1i(1,1)*A2i(1,1) + A1i(1,2)*A2i(2,1) + A1i(1,3)*A2i(3,1)
    + A1i(1,4)*A2i(4,1) + A1i(1,5)*A2i(5,1) + A1i(1,6)*A2i(6,1);
  ret(2) = A1i(2,1)*A2i(1,1) + A1i(2,2)*A2i(2,1) + A1i(2,3)*A2i(3,1)
    + A1i(2,4)*A2i(4,1) + A1i(2,5)*A2i(5,1) + A1i(2,6)*A2i(6,1);
  ret(3) = A1i(3,1)*A2i(1,1) + A1i(3,2)*A2i(2,1) + A1i(3,3)*A2i(3,1)
    + A1i(3,4)*A2i(4,1) + A1i(3,5)*A2i(5,1) + A1i(3,6)*A2i(6,1);
  ret(4) = A1i(4,1)*A2i(1,1) + A1i(4,2)*A2i(2,1) + A1i(4,3)*A2i(3,1)
    + A1i(4,4)*A2i(4,1) + A1i(4,5)*A2i(5,1) + A1i(4,6)*A2i(6,1);
  ret(5) = A1i(5,1)*A2i(1,1) + A1i(5,2)*A2i(2,1) + A1i(5,3)*A2i(3,1)
    + A1i(5,4)*A2i(4,1) + A1i(5,5)*A2i(5,1) + A1i(5,6)*A2i(6,1);
  ret(6) = A1i(6,1)*A2i(1,1) + A1i(6,2)*A2i(2,1) + A1i(6,3)*A2i(3,1)
    + A1i(6,4)*A2i(4,1) + A1i(6,5)*A2i(5,1) + A1i(6,6)*A2i(6,1);
  return ret;
}

template<typename T, typename Impl2>
OpenFDM_FORCE_INLINE
Vector<T,6>
operator*(const SymMatrix<T,6>& A1,
          const MatrixRValue<Impl2,6,1>& A2)
{
  const T* sptr = A1.find(1,1);
  const Impl2& A2i = A2.asImpl();

  T v1 = A2i(1,1);
  T v2 = A2i(2,1);
  T v3 = A2i(3,1);
  T v4 = A2i(4,1);
  T v5 = A2i(5,1);
  T v6 = A2i(6,1);

  Vector<T,6> ret;
  ret(1) = v1*sptr[0];
  ret(2) = v1*sptr[1];
  ret(1) += v2*sptr[1];
  ret(3) = v1*sptr[2];
  ret(1) += v3*sptr[2];
  ret(4) = v1*sptr[3];
  ret(1) += v4*sptr[3];
  ret(5) = v1*sptr[4];
  ret(1) += v5*sptr[4];
  ret(6) = v1*sptr[5];
  ret(1) += v6*sptr[5];

  ret(2) += v2*sptr[6];
  ret(3) += v2*sptr[7];
  ret(2) += v3*sptr[7];
  ret(4) += v2*sptr[8];
  ret(2) += v4*sptr[8];
  ret(5) += v2*sptr[9];
  ret(2) += v5*sptr[9];
  ret(6) += v2*sptr[10];
  ret(2) += v6*sptr[10];

  ret(3) += v3*sptr[11];
  ret(4) += v3*sptr[12];
  ret(3) += v4*sptr[12];
  ret(5) += v3*sptr[13];
  ret(3) += v5*sptr[13];
  ret(6) += v3*sptr[14];
  ret(3) += v6*sptr[14];

  ret(4) += v4*sptr[15];
  ret(5) += v4*sptr[16];
  ret(4) += v5*sptr[16];
  ret(6) += v4*sptr[17];
  ret(4) += v6*sptr[17];

  ret(5) += v5*sptr[18];
  ret(6) += v5*sptr[19];
  ret(5) += v6*sptr[19];

  ret(6) += v6*sptr[20];

  return ret;
}

template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,m1,n1>
max(const MatrixRValue<Impl1,m1,n1>& A1,
    const MatrixRValue<Impl2,m2,n2>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  size_type rows = A1i.rows();
  size_type cols = A1i.cols();
  SizeCheck<m1,m2>::Equal(rows, A2i.rows());
  SizeCheck<n1,n2>::Equal(cols, A2i.cols());

  Matrix<typename Impl1::value_type,m1,n1> ret(rows, cols);
  size_type i, j;
  for (j = 1; j <= cols; ++j)
    for (i = 1; i <= rows; ++i)
      ret(i, j) = max(A1i(i, j), A2i(i, j));

  return ret;
}

template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,m1,n1>
min(const MatrixRValue<Impl1,m1,n1>& A1,
    const MatrixRValue<Impl2,m2,n2>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  size_type rows = A1i.rows();
  size_type cols = A1i.cols();
  SizeCheck<m1,m2>::Equal(rows, A2i.rows());
  SizeCheck<n1,n2>::Equal(cols, A2i.cols());

  Matrix<typename Impl1::value_type,m1,n1> ret(rows, cols);
  size_type i, j;
  for (j = 1; j <= cols; ++j)
    for (i = 1; i <= rows; ++i)
      ret(i, j) = min(A1i(i, j), A2i(i, j));

  return ret;
}

#endif

} // namespace LinAlg

} // namespace OpenFDM

#endif
