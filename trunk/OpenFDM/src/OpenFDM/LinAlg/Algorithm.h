/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinAlgAlgorithm_H
#define OpenFDM_LinAlgAlgorithm_H

namespace OpenFDM {

namespace LinAlg {

/// Return the number of rows of a matrix.
template<typename Impl1, size_type m1, size_type n1>
OpenFDM_FORCE_INLINE
size_type
rows(const MatrixRValue<Impl1,m1,n1>& A)
{ return m1; }

/// Return the number of rows of a matrix.
template<typename Impl1, size_type n1>
OpenFDM_FORCE_INLINE
size_type
rows(const MatrixRValue<Impl1,0,n1>& A)
{ return A.asImpl().rows(); }

/// Return the number of columns of a matrix.
template<typename Impl1, size_type m1, size_type n1>
OpenFDM_FORCE_INLINE
size_type
cols(const MatrixRValue<Impl1,m1,n1>& A)
{ return n1; }

/// Return the number of columns of a matrix.
template<typename Impl1, size_type m1>
OpenFDM_FORCE_INLINE
size_type
cols(const MatrixRValue<Impl1,m1,0>& A)
{ return A.asImpl().cols(); }

typedef Vector2<size_type> Size;

/// Return the size a matrix in a size_type 2 vector.
template<typename Impl1, size_type m1, size_type n1>
OpenFDM_FORCE_INLINE
Size
size(const MatrixRValue<Impl1,m1,n1>& A)
{ return Vector2<size_type>(m1, n1); }

template<typename Impl1, size_type m1>
OpenFDM_FORCE_INLINE
Size
size(const MatrixRValue<Impl1,m1,0>& A)
{ return Vector2<size_type>(m1, A.asImpl().cols()); }

template<typename Impl1, size_type n1>
OpenFDM_FORCE_INLINE
Size
size(const MatrixRValue<Impl1,0,n1>& A)
{ return Vector2<size_type>(A.asImpl().rows(), n1); }

template<typename Impl1>
OpenFDM_FORCE_INLINE
Size
size(const MatrixRValue<Impl1,0,0>& A)
{ return Vector2<size_type>(A.asImpl().rows(), A.asImpl().cols()); }


template<typename Impl1, size_type n1, typename Impl2, size_type n2>
OpenFDM_FORCE_INLINE
typename Impl1::value_type
dot(const MatrixRValue<Impl1,n1,1>& v1, const MatrixRValue<Impl2,n2,1>& v2)
{
  typedef typename Impl1::value_type value_type;

  const Impl1& v1i = v1.asImpl();
  const Impl2& v2i = v2.asImpl();

  SizeCheck<n1,n2>::Equal(v1i.rows(), v2i.rows());

  value_type d = static_cast<value_type>(0);

  size_type rows = v1i.rows();

  size_type i;
  for (i = 0; i < rows; ++i)
    d += v1i(i, 0) * v2i(i, 0);

  return d;
}

template<typename Impl, size_type n>
OpenFDM_FORCE_INLINE
typename Impl::value_type
product(const MatrixRValue<Impl,n,1>& v)
{
  typedef typename Impl::value_type value_type;

  const Impl& vi = v.asImpl();

  value_type p = static_cast<value_type>(1);

  size_type rows = vi.rows();
  for (size_type i = 0; i < rows; ++i)
    p *= vi(i, 0);

  return p;
}

template<typename Impl, size_type n>
OpenFDM_FORCE_INLINE
typename Impl::value_type
norm(const MatrixRValue<Impl,n,1>& v)
{
  typedef typename Impl::value_type value_type;

  const Impl& vi = v.asImpl();

  value_type nrm = static_cast<value_type>(0);

  size_type rows = vi.rows();
  for (size_type i = 0; i < rows; ++i) {
    value_type tmp = vi(i, 0);
    nrm += tmp*tmp;
  }

  return sqrt(nrm);
}

template<typename Impl, size_type n>
OpenFDM_FORCE_INLINE
Vector<typename Impl::value_type,n>
normalize(const MatrixRValue<Impl,n,1>& v)
{
  typedef typename Impl::value_type value_type;
  value_type nrm = norm(v);
  if (fabs(nrm) <= Limits<real_type>::safe_min())
    return Vector<value_type,n>(v);
  else
    return Vector<value_type,n>((1/nrm)*v);
}

//     P = `1'
//           1-norm, the largest column sum of the absolute values of A.

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
typename Impl::value_type
norm1(const MatrixRValue<Impl,m,n>& A)
{
  typedef typename Impl::value_type value_type;

  const Impl& mi = A.asImpl();

  value_type nrm = static_cast<value_type>(0);

  size_type rows = mi.rows();
  size_type cols = mi.cols();
  for (size_type j = 0; j < cols; ++j) {
    value_type sum = static_cast<value_type>(0);
    for (size_type i = 0; i < rows; ++i)
      sum += fabs(mi(i, j));
    if (nrm < sum)
      nrm = sum;
  }

  return nrm;
}

//     P = `Inf'
//           Infinity norm, the largest row sum of the absolute values of
//           A.

template<typename Impl, size_type m, size_type n>
OpenFDM_FORCE_INLINE
typename Impl::value_type
normInf(const MatrixRValue<Impl,m,n>& A)
{
  typedef typename Impl::value_type value_type;

  const Impl& mi = A.asImpl();

  value_type nrm = static_cast<value_type>(0);

  size_type rows = mi.rows();
  size_type cols = mi.cols();
  for (size_type i = 0; i < rows; ++i) {
    value_type sum = static_cast<value_type>(0);
    for (size_type j = 0; j < cols; ++j)
      sum += fabs(mi(i, j));
    nrm = nrm < sum ? sum : nrm;
  }

  return nrm;
}

template<typename Impl, size_type n>
OpenFDM_FORCE_INLINE
size_type
maxIndex(const MatrixRValue<Impl,n,1>& v)
{
  typedef typename Impl::value_type value_type;

  const Impl& vi = v.asImpl();

  size_type idx = 0;

  value_type maximum = static_cast<value_type>(0);

  size_type rows = vi.rows();
  for (size_type i = 0; i < rows; ++i) {
    value_type absval = fabs(vi(i, 0));
    if (maximum < absval) {
      maximum = absval;
      idx = i;
    }
  }

  return idx;
}

/**  Comparison operator.

     @param A1 first matrix to compare.
     @param A2 second matrix to compare.

     Returns true if both matrices are exactly the same.
*/
template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
bool
operator==(const MatrixRValue<Impl1,m1,n1>& A1,
           const MatrixRValue<Impl2,m2,n2>& A2)
{
  const Impl1& A1i = A1.asImpl();
  const Impl2& A2i = A2.asImpl();

  size_type rows = A1i.rows();
  if (rows != A2i.rows())
    return false;
  size_type cols = A1i.cols();
  if (cols != A2i.cols())
    return false;

  for (size_type j = 0; j < cols; ++j)
    for (size_type i = 0; i < rows; ++i)
      if (A1i(i,j) != A2i(i,j))
        return false;

  return true;
}

/**  Comparison operator.

     @param A1 first matrix to compare.
     @param A2 second matrix to compare.

     Returns true if both matrices are not exactly the same.
*/
template<typename Impl1, size_type m1, size_type n1,
         typename Impl2, size_type m2, size_type n2>
OpenFDM_FORCE_INLINE
bool
operator!=(const MatrixRValue<Impl1,m1,n1>& A1,
           const MatrixRValue<Impl2,m2,n2>& A2)
{ return ! (A1 == A2); }

/**  Comparison operator.

     @param V1 first vector to compare.
     @param V2 second vector to compare.
     @param atol absolute tolerance.
     @param rtol relative tolerance.

     Returns true if both vectors are the same up to atol and rtol.
*/
template<typename Impl1, size_type m1,
         typename Impl2, size_type m2>
OpenFDM_FORCE_INLINE
bool
equal(const MatrixRValue<Impl1,m1,1>& V1,
      const MatrixRValue<Impl2,m2,1>& V2,
      typename Impl1::value_type atol,
      typename Impl1::value_type rtol)
{
  typedef typename Impl1::value_type value_type;

  OpenFDMLinAlgAssert(atol != static_cast<value_type>(0));

  const Impl1& V1i = V1.asImpl();
  const Impl2& V2i = V2.asImpl();

  size_type rows = V1i.rows();
  SizeCheck<m1,m2>::Equal(rows, V2i.rows());

  value_type nrmd = static_cast<value_type>(0);
  for (size_type i = 0; i < rows; ++i) {
    value_type v1 = V1i(i);
    value_type v2 = V2i(i);
    value_type d = (v1 - v2)/(atol + rtol*max(fabs(v1), fabs(v2)));
    nrmd += d*d;
  }

  return nrmd <= static_cast<value_type>(rows);
}

/**  Scaled difference of vectors.

     @param V1 first vector to compare.
     @param V2 second vector to compare.
     @param atol absolute tolerance.
     @param rtol relative tolerance.
*/
template<typename Impl1, size_type m1,
         typename Impl2, size_type m2>
OpenFDM_FORCE_INLINE
typename Impl1::value_type
scaledDiff(const MatrixRValue<Impl1,m1,1>& V1,
           const MatrixRValue<Impl2,m2,1>& V2,
           typename Impl1::value_type atol,
           typename Impl1::value_type rtol)
{
  typedef typename Impl1::value_type value_type;

  OpenFDMLinAlgAssert(atol != static_cast<value_type>(0));

  const Impl1& V1i = V1.asImpl();
  const Impl2& V2i = V2.asImpl();

  size_type rows = V1i.rows();
  SizeCheck<m1,m2>::Equal(rows, V2i.rows());

  value_type nrmd = static_cast<value_type>(0);
  for (size_type i = 0; i < rows; ++i) {
    value_type v1 = V1i(i, 0);
    value_type v2 = V2i(i, 0);
    value_type d = (v1 - v2)/(atol + rtol*max(fabs(v1), fabs(v2)));
    nrmd += d*d;
  }

  return sqrt(nrmd/static_cast<value_type>(rows));
}

/**  Scaled difference of vectors.

     @param V1 first vector to compare.
     @param V2 second vector to compare.
     @param atol absolute tolerance.
     @param rtol relative tolerance.
*/
template<typename Impl1, size_type m1,
         typename Impl2, size_type m2>
OpenFDM_FORCE_INLINE
typename Impl1::value_type
scaledErr(const MatrixRValue<Impl1,m1,1>& scale,
          const MatrixRValue<Impl2,m2,1>& err,
          typename Impl1::value_type atol,
          typename Impl1::value_type rtol)
{
  typedef typename Impl1::value_type value_type;

  OpenFDMLinAlgAssert(atol != static_cast<value_type>(0));

  const Impl1& scalei = scale.asImpl();
  const Impl2& erri = err.asImpl();

  size_type rows = scalei.rows();
  SizeCheck<m1,m2>::Equal(rows, erri.rows());

  value_type nrmd = static_cast<value_type>(0);
  for (size_type i = 0; i < rows; ++i) {
    value_type s = scalei(i, 0);
    value_type e = erri(i, 0);
    value_type d = e/(atol + rtol*max(max(fabs(s), fabs(s+e)), fabs(s-e)));
    nrmd += d*d;
  }

  return sqrt(nrmd/static_cast<value_type>(rows));
}

/** Comparison function.

    @param V1 vector.
    @param V2 vector.
    @param tol tolerance.

    Returns true if both vectors are the same up to relative tolerance tol.
*/
template<typename Impl1, size_type m1,
         typename Impl2, size_type m2>
OpenFDM_FORCE_INLINE
bool
equal(const MatrixRValue<Impl1,m1,1>& V1,
      const MatrixRValue<Impl2,m2,1>& V2,
      typename Impl1::value_type tol)
{
  typedef typename Impl1::value_type value_type;

  const Impl1& V1i = V1.asImpl();
  const Impl2& V2i = V2.asImpl();

  size_type rows = V1i.rows();
  SizeCheck<m1,m2>::Equal(rows, V2i.rows());

  value_type nrmv1 = static_cast<value_type>(0);
  value_type nrmv2 = static_cast<value_type>(0);
  value_type nrmd = static_cast<value_type>(0);
  for (size_type i = 0; i < rows; ++i) {
    value_type v1 = V1i(i);
    value_type v2 = V2i(i);
    value_type d = v1 - v2;
    nrmv1 += v1*v1;
    nrmv2 += v2*v2;
    nrmd += d*d;
  }

  return nrmd <= static_cast<value_type>(rows)*tol*tol*max(nrmv1, nrmv2);
}

/** Comparison function.

    @param u vector.
    @param v vector.

    Returns true if both vectors are the same up to roundoff.
*/
template<typename Impl1, size_type m1,
         typename Impl2, size_type m2>
OpenFDM_FORCE_INLINE
bool
equal(const MatrixRValue<Impl1,m1,1>& u,
      const MatrixRValue<Impl2,m2,1>& v)
{
  typedef typename Impl1::value_type value_type;
  value_type eps = Limits<value_type>::epsilon();
  return equal(u, v, 8*eps);
}

/** inf check.

    @param v vector.

    Returns true if all entries are finite.
*/
template<typename Impl, size_type m>
OpenFDM_FORCE_INLINE
bool
isFinite(const MatrixRValue<Impl,m,1>& v)
{
  typedef typename Impl::value_type value_type;
  const Impl& vi = v.asImpl();

  size_type rows = vi.rows();
  for (size_type i = 0; i < rows; ++i) {
    value_type val = vi(i, 0);
    if (!isfinite(val))
      return false;
  }

  return true;
}

/** Cross product multiplication.

    @param u vector to multiply.
    @param v vector to multiply with.
    @return The resulting vector from the cross product multiplication.

    Compute and return the cross product of the current vector with
    the given argument.
 */
template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
Vector<typename Impl1::value_type,3>
cross(const MatrixRValue<Impl1,3,1>& u, const MatrixRValue<Impl2,3,1>& v)
{
  const Impl1& ui = u.asImpl();
  const Impl2& vi = v.asImpl();

  Vector<typename Impl1::value_type,3> ret;
  ret(0,0) = ui(1,0)*vi(2,0) - ui(2,0)*vi(1,0);
  ret(1,0) = ui(2,0)*vi(0,0) - ui(0,0)*vi(2,0);
  ret(2,0) = ui(0,0)*vi(1,0) - ui(1,0)*vi(0,0);
  return ret;
}

/** Cross product multiplication.

    @param u matrix to multiply.
    @param v vector to multiply with.
    @return The resulting vector from the cross product multiplication.

    Compute and return the cross product of the current vector with
    the given argument.
    \f[
      v \times [u x]
    \f]
 */
template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,3,3>
cross(const MatrixRValue<Impl1,3,3>& u, const MatrixRValue<Impl2,3,1>& v)
{
  const Impl1& ui = u.asImpl();
  const Impl2& vi = v.asImpl();

  Matrix<typename Impl1::value_type,3,3> ret;

  ret(0,0) = vi(2,0)*ui(0,1) - vi(1,0)*ui(0,2);
  ret(1,0) = vi(2,0)*ui(1,1) - vi(1,0)*ui(1,2);
  ret(2,0) = vi(2,0)*ui(2,1) - vi(1,0)*ui(2,2);

  ret(0,1) = vi(0,0)*ui(0,2) - vi(2,0)*ui(0,0);
  ret(1,1) = vi(0,0)*ui(1,2) - vi(2,0)*ui(1,0);
  ret(2,1) = vi(0,0)*ui(2,2) - vi(2,0)*ui(2,0);

  ret(0,2) = vi(1,0)*ui(0,0) - vi(0,0)*ui(0,1);
  ret(1,2) = vi(1,0)*ui(1,0) - vi(0,0)*ui(1,1);
  ret(2,2) = vi(1,0)*ui(2,0) - vi(0,0)*ui(2,1);

  return ret;
}

/** Cross product multiplication.

    @param u vector to multiply.
    @param v matrix to multiply with.
    @return The resulting matrix from the column wise cross product
    multiplication.

    Compute and return the column wise cross product of the vector u with
    the matrix v.
    \f[
      [u x] \times v
    \f]
 */
template<typename Impl1, typename Impl2>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,3,3>
cross(const MatrixRValue<Impl1,3,1>& u, const MatrixRValue<Impl2,3,3>& v)
{
  const Impl1& ui = u.asImpl();
  const Impl2& vi = v.asImpl();

  Matrix<typename Impl1::value_type,3,3> ret;

  ret(0,0) = ui(1,0)*vi(2,0) - ui(2,0)*vi(1,0);
  ret(1,0) = ui(2,0)*vi(0,0) - ui(0,0)*vi(2,0);
  ret(2,0) = ui(0,0)*vi(1,0) - ui(1,0)*vi(0,0);

  ret(0,1) = ui(1,0)*vi(2,1) - ui(2,0)*vi(1,1);
  ret(1,1) = ui(2,0)*vi(0,1) - ui(0,0)*vi(2,1);
  ret(2,1) = ui(0,0)*vi(1,1) - ui(1,0)*vi(0,1);

  ret(0,2) = ui(1,0)*vi(2,2) - ui(2,0)*vi(1,2);
  ret(1,2) = ui(2,0)*vi(0,2) - ui(0,0)*vi(2,2);
  ret(2,2) = ui(0,0)*vi(1,2) - ui(1,0)*vi(0,2);

  return ret;
}

/** Cross product multiplication.
 */
template<typename Impl1>
OpenFDM_FORCE_INLINE
Matrix<typename Impl1::value_type,3,3>
cross(const MatrixRValue<Impl1,3,1>& u)
{
  const Impl1& ui = u.asImpl();
  Matrix<typename Impl1::value_type,3,3> ret;
  ret(0,0) =        0;  ret(0,1) = -ui(2,0);  ret(0,2) =  ui(1,0);
  ret(1,0) =  ui(2,0);  ret(1,1) =        0;  ret(1,2) = -ui(0,0);
  ret(2,0) = -ui(1,0);  ret(2,1) =  ui(0,0);  ret(2,2) =        0;
  return ret;
}

template<typename T, size_type dim1>
OpenFDM_FORCE_INLINE
void
l_substitute(const Matrix<T,dim1,dim1>& A, Vector<T,dim1>& v)
{
  typedef T value_type;

  size_type rows = A.rows();
  size_type cols = A.cols();
  for (size_type i = 0; i < rows-1; ++i) {
    if (v(i) != static_cast<value_type>(0))
      v(Range(i+1, cols-1)) -= v(i)*A(Range(i+1, cols-1),i);
  }
}

template<typename T, size_type dim1, size_type dim2>
OpenFDM_FORCE_INLINE
void
l_substitute(const Matrix<T,dim1,dim1>& A, Matrix<T,dim1,dim2>& v)
{
  typedef T value_type;

  size_type rows = A.rows();
  size_type cols = A.cols();
  size_type vcols = v.cols();
  for (size_type i = 0; i < rows-1; ++i) {
    for (size_type j = 0; j < vcols; ++j) {
      if (v(i,j) != static_cast<value_type>(0))
        v(Range(i+1, cols-1),j) -= v(i,j)*A(Range(i+1, cols-1),i);
    }
  }
}

template<typename T,size_type dim1,size_type dim2>
OpenFDM_FORCE_INLINE
void
u_substitute(const Matrix<T,dim1,dim2>& A, Vector<T,dim1>& v)
{
  typedef T value_type;

  size_type cols = A.cols();
  for (size_type ii = cols; 0 < ii; --ii) {
    size_type i = ii - 1;
    if (v(i) != static_cast<value_type>(0)) {
      value_type Aii = A(i,i);
      // If the matrix is exactly singular, compute the solution where the
      // righthandside is projected into the image of the matrix.
      if (fabs(Aii) <= fabs(v(i))*Limits<value_type>::safe_min()) {
        v(i) = static_cast<value_type>(0);
      } else {
        v(i) /= Aii;
        if (0 < i)
          v(Range(0,i-1)) -= v(i)*A(Range(0,i-1),i);
      }
    }
  }
}

template<typename T,size_type dim1,size_type dim2,size_type dim3>
OpenFDM_FORCE_INLINE
void
u_substitute(const Matrix<T,dim1,dim2>& A, Matrix<T,dim1,dim3>& v)
{
  typedef T value_type;

  size_type cols = A.cols();
  size_type vcols = v.cols();
  for (size_type ii = cols; 0 < ii; --ii) {
    size_type i = ii-1;
    for (size_type j = 0; j < vcols; ++j) {
      if (v(i,j) != static_cast<value_type>(0)) {
        value_type Aii = A(i,i);
        // If the matrix is exactly singular, compute the solution where the
        // righthandside is projected into the image of the matrix.
        if (fabs(Aii) <= fabs(v(i,j))*Limits<value_type>::safe_min()) {
          v(i,j) = static_cast<value_type>(0);
        } else {
          v(i,j) /= Aii;
          if (0 < i)
            v(Range(0,i-1),j) -= v(i,j)*A(Range(0,i-1),i);
        }
      }
    }
  }
}

template<typename T, size_type dim1>
OpenFDM_FORCE_INLINE
void
p_substitute(const Vector<size_type,dim1>& perm, Vector<T,dim1>& v)
{
  typedef T value_type;

  size_type rows = v.rows();
  for (size_type i = 0; i < rows; ++i) {
    size_type ip = perm(i);
    if (ip != i) {
      value_type tmp = v(i);
      v(i) = v(ip);
      v(ip) = tmp;
    }
  }
}

template<typename T, size_type dim1, size_type dim2>
OpenFDM_FORCE_INLINE
void
p_substitute(const Vector<size_type,dim1>& perm, Matrix<T,dim1,dim2>& v)
{
  typedef T value_type;

  size_type rows = v.rows();
  size_type cols = v.cols();
  for (size_type i = 0; i < rows; ++i) {
    size_type ip = perm(i);
    if (ip != i) {
      for (size_type j = 0; j < cols; ++j) {
        value_type tmp = v(i,j);
        v(i,j) = v(ip,j);
        v(ip,j) = tmp;
      }
    }
  }
}

template<typename T, size_type dim1>
void
lu_substitute(const Matrix<T,dim1,dim1>& A, Vector<T,dim1>& v)
{
  // Solve L^{-1}v
  l_substitute(A, v);
  // Solve U^{-1}v
  u_substitute(A, v);
}

template<typename T, size_type dim1, size_type dim2>
void
lu_substitute(const Matrix<T,dim1,dim1>& A, Matrix<T,dim1,dim2>& v)
{
  // Solve L^{-1}v
  l_substitute(A, v);
  // Solve U^{-1}v
  u_substitute(A, v);
}

template<typename T, size_type dim1>
void
lu_substitute(const Matrix<T,dim1,dim1>& A, const Vector<size_type,dim1>& perm,
              Vector<T,dim1>& v)
{
  // Apply permutation matrix
  p_substitute(perm, v);
  // Solve L^{-1}v
  l_substitute(A, v);
  // Solve U^{-1}v
  u_substitute(A, v);
}

template<typename T, size_type dim1, size_type dim2>
void
lu_substitute(const Matrix<T,dim1,dim1>& A, const Vector<size_type,dim1>& perm,
              Matrix<T,dim1,dim2>& v)
{
  // Apply permutation matrix
  p_substitute(perm, v);
  // Solve L^{-1}v
  l_substitute(A, v);
  // Solve U^{-1}v
  u_substitute(A, v);
}


// Compute the lu facorization of the matrix.
// Does only work for square matrices.
// Overwrites A with the factors.
template<typename T, size_type dim1, size_type dim2>
bool
lu_factorize(Matrix<T,dim1,dim2>& A)
{
  typedef T value_type;

  bool nonsingular = true;

  size_type m = A.rows();
  size_type n = A.cols();
  for (size_type j = 0; j < n; ++j) {
    // The matrix is exactly singular.
    // Hmm, should read like that??
    // if (fabs(A(j,j)) <= normInf(A(Range(j+1, m-1), j))*Limits<value_type>::safe_min())
    if (fabs(A(j,j)) <= Limits<value_type>::safe_min())
      nonsingular = false;
    else {
      if (j < n-1) {
        // Compute elements J+1:M of J-th column.
        A(Range(j+1, m-1), j) *= static_cast<value_type>(1)/A(j,j);
        
        A(Range(j+1, m-1), Range(j+1, n-1))
          -= A(Range(j+1,m-1),j)*A(j, Range(j+1, n-1));
      }
    }
  }

  return nonsingular;
}

template<typename T, size_type dim1, size_type dim2>
bool
lu_factorize(Matrix<T,dim1,dim2>& A, Vector<size_type,dim1>& perm)
{
  typedef T value_type;

  bool nonsingular = true;

  size_type m = A.rows();
  size_type n = A.cols();

  perm.resize(n, 1);

  for (size_type j = 0; j < n; ++j) {
    size_type jp = j + maxIndex(A(Range(j, m-1), j));
    perm(j) = jp;

    // The matrix is exactly singular.
    if (fabs(A(jp,j)) <= Limits<value_type>::safe_min())
      nonsingular = false;
    else {
      if (jp != j) {
        // FIXME ...
//         RangeExpr<Matrix<T,dim1,dim2>,1,0> re1 = A(j, Range(1, n-1));
//         RangeExpr<Matrix<T,dim1,dim2>,1,0> re2 = A(jp, Range(1, n-1));
        MatrixPointerExpr<Matrix<T,dim1,dim2>,1,0> re1 = A(j, Range(0, n-1));
        MatrixPointerExpr<Matrix<T,dim1,dim2>,1,0> re2 = A(jp, Range(0, n-1));
        swap(re1, re2);
      }
    
      if (j < n-1) {
        // Compute elements J+1:M of J-th column.
        A(Range(j+1, m-1), j) *= static_cast<value_type>(1)/A(j, j);
        
        A(Range(j+1, m-1), Range(j+1, n-1))
          -= A(Range(j+1,m-1),j)*A(j, Range(j+1, n-1));
      }
    }
  }

  return nonsingular;
}


// Solve using the qr facorization of the matrix.
// Overwrites x with the solution.
// If the matrix is rectangular, the upper part of x contains the least
// squares solution and the lower part contains the least squares error.
template<typename T, size_type dim1, size_type dim2>
bool
q_substitute(const Matrix<T,dim1,dim2>& A, const Vector<T,dim2>& beta,
            Vector<T,dim1>& x)
{
  typedef T value_type;

  bool nonsingular = true;

  size_type m = A.rows();
  size_type n = A.cols();
  
  for (size_type j = 0; j < n; ++j) {
    // See Golub, van Loan: Matrix Computations, pp 210

    if (j < m-1) {
      Vector<T> v(A(Range(j, m-1), j));
      v(0) = 1;
      x(Range(j, m-1)) -= (beta(j)*dot(v, x(Range(j, m-1))))*v;
    }
  }

  return nonsingular;
}

template<typename T, size_type dim1, size_type dim2>
void
qr_substitute(const Matrix<T,dim1,dim2>& A, const Vector<T,dim2>& beta,
              Vector<T,dim1>& v)
{
  // Solve Q^t v
  q_substitute(A, beta, v);
  // Solve U^{-1} v
  u_substitute(A, v);
}


template<typename Impl, size_type dim>
OpenFDM_FORCE_INLINE
void
qr_reflector(typename Impl::value_type& alpha, MatrixLValue<Impl,dim,1>& x_,
             typename Impl::value_type& tau)
{
  // Directly taken from the LAPACK routine DLARFG
  typedef typename Impl::value_type value_type;
  Impl& x = x_.asImpl();

  value_type xnorm = norm(x);
  if (xnorm == 0)
    tau = 0;
  else {
    value_type beta = sqrt(alpha*alpha + xnorm*xnorm);
    if (0 <= alpha)
      beta = -beta;

    value_type safmin = Limits<value_type>::safe_min();

    if (fabs(beta) < safmin) {
      value_type rsafmn = 1/safmin;
      size_type knt = 0;
      do {
        ++knt;
        x *= rsafmn;
        beta *= rsafmn;
        alpha *= rsafmn;
      } while (fabs(beta) < safmin);

      xnorm = norm(x);
      beta = sqrt(alpha*alpha + xnorm*xnorm);
      if (0 <= alpha)
        beta = -beta;
      tau = (beta-alpha)/beta;
      x *= 1/(alpha-beta);
      alpha = beta;
      for (; 0 < knt; --knt)
        alpha *= safmin;
    } else {
      tau = (beta-alpha)/beta;
      x *= 1/(alpha-beta);
      alpha = beta;
    }
  }
}

// Compute the qu facorization of the matrix.
// Overwrites A with the factors.
template<typename T, size_type dim1, size_type dim2>
bool
qr_factorize(Matrix<T,dim1,dim2>& A, Vector<T,dim2>& beta)
{
  typedef T value_type;

  bool nonsingular = true;

  size_type m = A.rows();
  size_type n = A.cols();
  
  // FIXME:
  beta.resize(n, 1);

  for (size_type j = 0; j < n; ++j) {
    // Compute the Householder vector v.
    // See Golub, van Loan: Matrix Computations, pp 210
    
//     RangeExpr<Matrix<T,dim1,dim2>,0,1> x = A(Range(j+1, m-1), j);
    MatrixPointerExpr<Matrix<T,dim1,dim2>,0,1> x = A(Range(j+1, m-1), j);

    qr_reflector(A(j, j), x, beta(j));

    // Check if that thing is singular.
    value_type Ajj = A(j, j);
    if (fabs(Ajj) <= Limits<value_type>::safe_min())
      nonsingular = false;

    A(j, j) = 1;
    // FIXME:
//     RangeExpr<Matrix<T,dim1,dim2>,0,1> v = A(Range(j, m-1), j);
    MatrixPointerExpr<Matrix<T,dim1,dim2>,0,1> v = A(Range(j, m-1), j);

    // FIXME:!!!!!
    Matrix<T> work(beta(j)*trans(v)*A(Range(j, m-1), Range(j+1, n-1)));
    A(Range(j, m-1), Range(j+1, n-1)) -= v*work;

    A(j, j) = Ajj;
  }

  return nonsingular;
}

struct LUTag {};
struct QRTag {};

template<typename T,size_type m,size_type n,typename DecompTag>
class MatrixFactors;

template<typename T,size_type n>
class MatrixFactors<T,n,n,LUTag> {
public:
  OpenFDM_FORCE_INLINE
  MatrixFactors(void)
  {}
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  MatrixFactors(const MatrixRValue<Impl,n,n>& A)
    : lu_factors_(A)
  { factorize(); }

  template<typename Impl>
  OpenFDM_FORCE_INLINE
  MatrixFactors& operator=(const MatrixRValue<Impl,n,n>& A)
  { lu_factors_ = A; factorize(); return *this; }

  const Matrix<T,n,n>& data(void) const
  { return lu_factors_; }
  Matrix<T,n,n>& data(void)
  { return lu_factors_; }

  OpenFDM_FORCE_INLINE
  bool factorize(void)
  { singular_ = ! lu_factorize(lu_factors_, perm_); return singular_; }

  OpenFDM_FORCE_INLINE
  bool singular(void) const
  { return singular_; }

  OpenFDM_FORCE_INLINE
  void inplaceSolve(Vector<T,n>& v) const
  { lu_substitute(lu_factors_, perm_, v); }

  template<typename Impl, size_type m>
  OpenFDM_FORCE_INLINE
  Matrix<T,n,m> solve(const MatrixRValue<Impl,n,m>& v) const
  { Matrix<T,n,m> ret(v); lu_substitute(lu_factors_, perm_, ret); return ret; }

  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Vector<T,n> solve(const MatrixRValue<Impl,n,1>& v) const
  { Vector<T,n> ret(v); lu_substitute(lu_factors_, perm_, ret); return ret; }

private:
  Matrix<T,n,n> lu_factors_;
  Vector<size_type,n> perm_;
  bool singular_;
};

template<typename T,size_type m,size_type n>
class MatrixFactors<T,m,n,QRTag> {
public:
  OpenFDM_FORCE_INLINE
  MatrixFactors(void)
  {}
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  MatrixFactors(const MatrixRValue<Impl,m,n>& A)
    : qr_factors_(A)
  { factorize(); }

  template<typename Impl>
  OpenFDM_FORCE_INLINE
  MatrixFactors& operator=(const MatrixRValue<Impl,m,n>& A)
  { qr_factors_ = A; factorize(); return *this; }

  const Matrix<T,m,n>& data(void) const
  { return qr_factors_; }
  Matrix<T,m,n>& data(void)
  { return qr_factors_; }

  OpenFDM_FORCE_INLINE
  bool factorize(void)
  { singular_ = ! qr_factorize(qr_factors_, beta_); return singular_; }

  OpenFDM_FORCE_INLINE
  bool singular(void) const
  { return singular_; }

  OpenFDM_FORCE_INLINE
  void inplaceSolve(Vector<T,m>& v) const
  { qr_substitute(qr_factors_, beta_, v); }

  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Vector<T,n> solve(const MatrixRValue<Impl,m,1>& v) const
  {
    Vector<T,m> tmp(v);
    qr_substitute(qr_factors_, beta_, tmp);
    return Vector<T,n>(tmp(Range(0, qr_factors_.cols()-1)));
  }

private:
  Matrix<T,m,n> qr_factors_;
  Vector<T,n> beta_;
  bool singular_;
};

} // namespace LinAlg

} // namespace OpenFDM

#endif
