/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinAlgContainer_H
#define OpenFDM_LinAlgContainer_H

namespace OpenFDM {

namespace LinAlg {

// A dynamic sized matrix container.
//  T is the value type for that one.
template<typename T, size_type rows_ = 0, size_type cols_ = 0>
class Matrix
  : public MatrixLValue<Matrix<T,rows_,cols_>,rows_,cols_> {
public:
  typedef T value_type;

  OpenFDM_FORCE_INLINE
  Matrix(void)
  { }
  OpenFDM_FORCE_INLINE
  Matrix(size_type i, size_type j)
  { resize(i, j); }
  OpenFDM_FORCE_INLINE
  Matrix(const Matrix& A)
  { assign(A); }
  template<typename Impl2, size_type m2, size_type n2>
  OpenFDM_FORCE_INLINE
  Matrix(const MatrixRValue<Impl2,m2,n2>& A)
  { assign(A); }
  OpenFDM_FORCE_INLINE
  ~Matrix(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return data_.rows(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return data_.cols(); }

  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return *data_.find(i, j); }
  /** Nonconst accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type& operator()(size_type i, size_type j)
  { return *data_.find(i, j); }

  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_.find(i, j); }
  /** Nonconst accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_.find(i, j); }

//   OpenFDM_FORCE_INLINE
//   RangeExpr<Matrix,0,0>
//   operator()(const Range& r1, const Range& r2)
//   { return RangeExpr<Matrix,0,0>(*this, r1, r2); }
//   OpenFDM_FORCE_INLINE
//   RangeExpr<Matrix,0,1>
//   operator()(const Range& r1, size_type j)
//   { return RangeExpr<Matrix,0,1>(*this, r1, Range(j)); }
//   OpenFDM_FORCE_INLINE
//   RangeExpr<Matrix,1,0>
//   operator()(size_type i, const Range& r2)
//   { return RangeExpr<Matrix,1,0>(*this, Range(i), r2); }


  OpenFDM_FORCE_INLINE
  MatrixPointerExpr<Matrix,0,0>
  operator()(const Range& r1, const Range& r2)
  { return MatrixPointerExpr<Matrix,0,0>(*this, r1, r2); }
  OpenFDM_FORCE_INLINE
  MatrixPointerExpr<Matrix,0,1>
  operator()(const Range& r1, size_type j)
  { return MatrixPointerExpr<Matrix,0,1>(*this, r1, Range(j)); }
  OpenFDM_FORCE_INLINE
  MatrixPointerExpr<Matrix,1,0>
  operator()(size_type i, const Range& r2)
  { return MatrixPointerExpr<Matrix,1,0>(*this, Range(i), r2); }

//   OpenFDM_FORCE_INLINE
//   ConstRangeExpr<Matrix,0,0>
//   operator()(const Range& r1, const Range& r2) const
//   { return ConstRangeExpr<Matrix,0,0>(*this, r1, r2); }
//   OpenFDM_FORCE_INLINE
//   ConstRangeExpr<Matrix,0,1>
//   operator()(const Range& r1, size_type j) const
//   { return ConstRangeExpr<Matrix,0,1>(*this, r1, Range(j)); }
//   OpenFDM_FORCE_INLINE
//   ConstRangeExpr<Matrix,1,0>
//   operator()(size_type i, const Range& r2) const
//   { return ConstRangeExpr<Matrix,1,0>(*this, Range(i), r2); }

  OpenFDM_FORCE_INLINE
  ConstMatrixPointerExpr<Matrix,0,0>
  operator()(const Range& r1, const Range& r2) const
  { return ConstMatrixPointerExpr<Matrix,0,0>(*this, r1, r2); }
  OpenFDM_FORCE_INLINE
  ConstMatrixPointerExpr<Matrix,0,1>
  operator()(const Range& r1, size_type j) const
  { return ConstMatrixPointerExpr<Matrix,0,1>(*this, r1, Range(j)); }
  OpenFDM_FORCE_INLINE
  ConstMatrixPointerExpr<Matrix,1,0>
  operator()(size_type i, const Range& r2) const
  { return ConstMatrixPointerExpr<Matrix,1,0>(*this, Range(i), r2); }

  /** Index function.
   */
  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  { return data_.index(i, j); }

  /** Resize to a given size.
   */
  OpenFDM_FORCE_INLINE
  bool resize(size_type i, size_type j)
  { return data_.resize(i, j); }
  template<typename Impl2, size_type m2, size_type n2>
  OpenFDM_FORCE_INLINE
  bool resize(const MatrixRValue<Impl2,m2,n2>& A)
  { return resize(A.asImpl().rows(), A.asImpl().cols()); }

  OpenFDM_FORCE_INLINE
  Matrix& operator=(const Matrix& mtrx)
  { assign(mtrx); return *this; }

  OpenFDM_FORCE_INLINE
  Matrix& operator*=(value_type scalar)
  { scalarMultiplyMatrix(*this, scalar); return *this; }

private:
  RectangularArray<T,rows_,cols_> data_;
};

// A dynamic sized matrix container.
//  T is the value type for that one.
template<typename T, size_type rows_ = 0>
class Vector
  : public MatrixLValue<Vector<T,rows_>,rows_,1> {
public:
  typedef T value_type;

  OpenFDM_FORCE_INLINE
  Vector(void)
  { }
  OpenFDM_FORCE_INLINE
  Vector(size_type i, size_type = 1)
  { resize(i); }
  OpenFDM_FORCE_INLINE
  Vector(const Vector& A)
  { directAssign(A); }
  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  Vector(const MatrixRValue<Impl2,m2,1>& A)
  { directAssign(A); }
  OpenFDM_FORCE_INLINE
  ~Vector(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return data_.size(); }
  OpenFDM_FORCE_INLINE
  size_type size(void) const
  { return data_.size(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return 1; }

  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i) const
  { return *data_.find(i); }
  /** Nonconst accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type& operator()(size_type i)
  { return *data_.find(i); }

  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return *data_.find(i); }
  /** Nonconst accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type& operator()(size_type i, size_type j)
  { return *data_.find(i); }

  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_.find(i); }
  /** Nonconst accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_.find(i); }

//   OpenFDM_FORCE_INLINE
//   RangeExpr<Vector,0,1>
//   operator()(const Range& r1)
//   { return RangeExpr<Vector,0,1>(*this, r1, Range(1)); }
  OpenFDM_FORCE_INLINE
  MatrixPointerExpr<Vector,0,1>
  operator()(const Range& r1)
  { return MatrixPointerExpr<Vector,0,1>(*this, r1, Range(1)); }

//   OpenFDM_FORCE_INLINE
//   ConstRangeExpr<Vector,0,1>
//   operator()(const Range& r1) const
//   { return ConstRangeExpr<Vector,0,1>(*this, r1, Range(1)); }

  OpenFDM_FORCE_INLINE
  ConstMatrixPointerExpr<Vector,0,1>
  operator()(const Range& r1) const
  { return ConstMatrixPointerExpr<Vector,0,1>(*this, r1, Range(1)); }

  /** Index function.
   */
  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  { return data_.index(i); }

  /** Resize to a given size.
   */
  OpenFDM_FORCE_INLINE
  bool resize(size_type i, size_type = 1)
  { return data_.resize(i); }
  template<typename Impl2, size_type m2, size_type n2>
  OpenFDM_FORCE_INLINE
  bool resize(const MatrixRValue<Impl2,m2,n2>& A)
  {
    SizeCheck<1,n2>::Equal(1, A.asImpl().cols());
    return resize(A.asImpl().rows());
  }

  OpenFDM_FORCE_INLINE
  Vector& operator*=(value_type scalar)
  { scalarMultiplyMatrix(*this, scalar); return *this; }

  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  Vector& operator+=(const MatrixRValue<Impl2,m2,1>& A)
  {
    const Impl2& Ai = A.asImpl();

    size_type r = Ai.rows();
    SizeCheck<rows_,m2>::Equal(r, rows());

    size_type i;
    for (i = 1; i <= r; ++i)
      (*this)(i) += Ai(i, 1);

    return *this;
  }

  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  Vector& operator-=(const MatrixRValue<Impl2,m2,1>& A)
  {
    const Impl2& Ai = A.asImpl();

    size_type r = Ai.rows();
    SizeCheck<rows_,m2>::Equal(r, rows());

    size_type i;
    for (i = 1; i <= r; ++i)
      (*this)(i) -= Ai(i, 1);

    return *this;
  }

  OpenFDM_FORCE_INLINE
  Vector& operator=(const Vector& Ai)
  {
    size_type r = Ai.rows();
    resize(r);
    SizeCheck<rows_,rows_>::Equal(r, rows());

    size_type i;
    for (i = 1; i <= r; ++i)
      (*this)(i) = Ai(i, 1);

    return *this;
  }

  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  Vector& operator=(const MatrixRValue<Impl2,m2,1>& A)
  {
    const Impl2& Ai = A.asImpl();

    size_type r = Ai.rows();
    resize(r);
    SizeCheck<rows_,m2>::Equal(r, rows());

    size_type i;
    for (i = 1; i <= r; ++i)
      (*this)(i) = Ai(i, 1);

    return *this;
  }


  /// All zero vector.
  static Vector zeros(size_type sz = Vector().size())
  {
    Vector ret(sz);
    size_type i;
    for (i = 1; i <= sz; ++i)
      ret(i) = 0;
    return ret;
  }

  /// idx-th unit vector.
  static Vector unit(size_type idx, size_type sz = Vector().size())
  {
    Vector ret = Vector::zeros(sz);
    OpenFDMLinAlgAssert(0 < idx && idx <= ret.size());
    ret(idx) = 1;
    return ret;
  }



private:
  LinearArray<T,rows_> data_;
};


// A generic matrix container.
//  T is the value type for that one.
//  S is the size template.
template<typename T,size_type rows_>
class SymMatrix
  : public MatrixLValue<SymMatrix<T,rows_>,rows_,rows_> {
public:
  typedef T value_type;

  OpenFDM_FORCE_INLINE
  SymMatrix(void)
  { }
  OpenFDM_FORCE_INLINE
  SymMatrix(size_type i, size_type j)
  { resize(i, j); }
  OpenFDM_FORCE_INLINE
  SymMatrix(const SymMatrix& A)
  {
    resize(A.rows(), A.cols());
    T* ptr = find(1,1);
    const T* Aptr = A.find(1,1);
    size_type arraylen = (rows()*(rows()+1))/2;
    size_type i;
    for (i = 0; i < arraylen; ++i)
      ptr[i] = Aptr[i];
  }
  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  SymMatrix(const MatrixRValue<Impl2,m2,m2>& A)
  { directAssign(A.asImpl()); }
  OpenFDM_FORCE_INLINE
  ~SymMatrix(void)
  { }

  /** Number of rows.
   */
  OpenFDM_FORCE_INLINE
  size_type rows(void) const
  { return data_.rows(); }
  /** Number of columns.
   */
  OpenFDM_FORCE_INLINE
  size_type cols(void) const
  { return data_.cols(); }

  /** Const accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type operator()(size_type i, size_type j) const
  { return *data_.find(i, j); }
  /** Nonconst accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type& operator()(size_type i, size_type j)
  { return *data_.find(i, j); }

  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_.find(i, j); }
  /** Nonconst accessor.
   */
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_.find(i, j); }

  OpenFDM_FORCE_INLINE
  RangeExpr<SymMatrix,0,0>
  operator()(const Range& r1, const Range& r2)
  { return RangeExpr<SymMatrix,0,0>(*this, r1, r2); }
  OpenFDM_FORCE_INLINE
  RangeExpr<SymMatrix,0,1>
  operator()(const Range& r1, size_type j)
  { return RangeExpr<SymMatrix,0,1>(*this, r1, Range(j)); }
  OpenFDM_FORCE_INLINE
  RangeExpr<SymMatrix,1,0>
  operator()(size_type i, const Range& r2)
  { return RangeExpr<SymMatrix,1,0>(*this, Range(i), r2); }

  OpenFDM_FORCE_INLINE
  ConstRangeExpr<SymMatrix,0,0>
  operator()(const Range& r1, const Range& r2) const
  { return ConstRangeExpr<SymMatrix,0,0>(*this, r1, r2); }
  OpenFDM_FORCE_INLINE
  ConstRangeExpr<SymMatrix,0,1>
  operator()(const Range& r1, size_type j) const
  { return ConstRangeExpr<SymMatrix,0,1>(*this, r1, Range(j)); }
  OpenFDM_FORCE_INLINE
  ConstRangeExpr<SymMatrix,1,0>
  operator()(size_type i, const Range& r2) const
  { return ConstRangeExpr<SymMatrix,1,0>(*this, Range(i), r2); }

  /** Index function.
   */
  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  { return data_.index(i, j); }

  /** Resize to a given size.
   */
  OpenFDM_FORCE_INLINE
  bool resize(size_type i, size_type j)
  { OpenFDMLinAlgAssert(i == j); return data_.resize(i); }
  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  bool resize(const MatrixRValue<Impl2,m2,m2>& A)
  { return resize(A.asImpl().rows()); }

  OpenFDM_FORCE_INLINE
  SymMatrix& operator=(const SymMatrix& A)
  {
#ifdef USE_EXPRESSIONS
    assign(A);
#else
    T* ptr = find(1,1);
    const T* Aptr = A.find(1,1);
    size_type arraylen = (rows()*(rows()+1))/2;
    size_type i;
    for (i = 0; i < arraylen; ++i)
      ptr[i] = Aptr[i];
#endif
    return *this;
  }

  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  SymMatrix& operator=(const MatrixRValue<Impl2,m2,m2>& A)
  { assign(A.asImpl()); return *this; }

  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  SymMatrix& operator+=(const MatrixRValue<Impl2,m2,m2>& A)
  { plusAssign(A.asImpl()); return *this; }

  OpenFDM_FORCE_INLINE
  SymMatrix& operator+=(const SymMatrix& A)
  {
#ifdef USE_EXPRESSIONS
    plusAssign(A.asImpl());
#else
    T* ptr = find(1,1);
    const T* Aptr = A.find(1,1);
    size_type arraylen = (rows()*(rows()+1))/2;
    size_type i;
    for (i = 0; i < arraylen; ++i)
      ptr[i] += Aptr[i];
#endif
    return *this;
  }

  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  SymMatrix& operator-=(const MatrixRValue<Impl2,m2,m2>& A)
  { minusAssign(A.asImpl()); return *this; }

  template<typename Impl2, size_type m2>
  OpenFDM_FORCE_INLINE
  SymMatrix& operator-=(const SymMatrix& A)
  {
#ifdef USE_EXPRESSIONS
    minusAssign(A);
#else
    T* ptr = find(1,1);
    const T* Aptr = A.find(1,1);
    size_type arraylen = (rows()*(rows()+1))/2;
    size_type i;
    for (i = 0; i < arraylen; ++i)
      ptr[i] -= Aptr[i];
#endif
    return *this;
  }

  OpenFDM_FORCE_INLINE
  SymMatrix& operator*=(value_type scalar)
  {
    T* ptr = find(1,1);
    size_type arraylen = (rows()*(rows()+1))/2;
    size_type i;
    for (i = 0; i < arraylen; ++i)
      ptr[i] *= scalar;
    return *this;
  }

private:
  template<typename Impl2>
  OpenFDM_FORCE_INLINE
  void assign(const Impl2& A)
  {
#ifdef USE_EXPRESSIONS
    size_type r = A.rows();
    size_type c = A.cols();
    SymMatrix tmp(r, c);
    size_type i, j;
    for (j = 1; j <= c; ++j) {
      for (i = j; i <= r; ++i) {
        tmp(i,j) = A(i,j);
      }
    }

    resize(r, c);
    SizeCheck<rows_,rows_>::Equal(rows(), A.rows());
    SizeCheck<rows_,rows_>::Equal(cols(), A.cols());

    for (j = 1; j <= c; ++j) {
      for (i = j; i <= r; ++i) {
        (*this)(i,j) = tmp(i,j);
      }
    }
#else
    directAssign(A);
#endif
  }
  template<typename Impl2>
  OpenFDM_FORCE_INLINE
  void directAssign(const Impl2& A)
  {
    size_type r = A.rows();
    size_type c = A.cols();

    resize(r, c);
    SizeCheck<rows_,rows_>::Equal(rows(), A.rows());
    SizeCheck<rows_,rows_>::Equal(cols(), A.cols());

    size_type i, j;
    for (j = 1; j <= c; ++j) {
      for (i = j; i <= r; ++i) {
        (*this)(i,j) = A(i,j);
      }
    }
  }
  template<typename Impl2>
  OpenFDM_FORCE_INLINE
  void plusAssign(const Impl2& A)
  {
    size_type r = A.rows();
    size_type c = A.cols();

    SizeCheck<rows_,rows_>::Equal(rows(), A.rows());
    SizeCheck<rows_,rows_>::Equal(cols(), A.cols());

    size_type i, j;
    for (j = 1; j <= c; ++j) {
      for (i = j; i <= r; ++i) {
        (*this)(i,j) += A(i,j);
      }
    }
  }
  template<typename Impl2>
  OpenFDM_FORCE_INLINE
  void minusAssign(const Impl2& A)
  {
    size_type r = A.rows();
    size_type c = A.cols();

    SizeCheck<rows_,rows_>::Equal(rows(), A.rows());
    SizeCheck<rows_,rows_>::Equal(cols(), A.cols());

    size_type i, j;
    for (j = 1; j <= c; ++j) {
      for (i = j; i <= r; ++i) {
        (*this)(i,j) -= A(i,j);
      }
    }
  }

  SymmetricArray<T,rows_> data_;
};

// A generic all zero matrix.
//  T is the value type for that one.
//  S is the size template.
template<typename, size_type, size_type>
class Zeros;

// A dynamic sized matrix container.
//  T is the value type for that one.
template<typename T>
class Zeros<T,0,0>
  : public MatrixRValue<Zeros<T,0,0>,0,0> {
public:
  typedef T value_type;

  OpenFDM_FORCE_INLINE
  Zeros(void)
    : rows_(0), cols_(0)
  { }
  OpenFDM_FORCE_INLINE
  Zeros(size_type i, size_type j)
    : rows_(0), cols_(0)
  { resize(i, j); }
  OpenFDM_FORCE_INLINE
  Zeros(const Zeros& mtrx)
    : rows_(0), cols_(0)
  { resize(mtrx.rows(), mtrx.cols()); }
  OpenFDM_FORCE_INLINE
  ~Zeros(void)
  { }

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
  { return static_cast<value_type>(0); }

  OpenFDM_FORCE_INLINE
  Zeros<T,0,0>
  operator()(const Range& r1, const Range& r2 = Range(1)) const
  { return Zeros<T,0,0>(r1.last - r1.first + 1, r2.last - r2.first + 1); }

  /** Resize to a given size.
   */
  OpenFDM_FORCE_INLINE
  bool resize(size_type i, size_type j)
  { rows_ = i; cols_ = j; return true; }
  template<typename Impl2, size_type m2, size_type n2>
  OpenFDM_FORCE_INLINE
  bool resize(const MatrixRValue<Impl2,m2,n2>& A)
  { return resize(A.asImpl().rows(), A.asImpl().cols()); }

  OpenFDM_FORCE_INLINE
  Zeros& operator=(const Zeros& mtrx)
  { resize(mtrx.rows(), mtrx.cols()); return *this; }

private:
  size_type rows_;
  size_type cols_;
};


// A dynamic sized matrix container.
//  T is the value type for that one.
template<typename T, size_type rows_, size_type cols_>
class Zeros
  : public MatrixRValue<Zeros<T,rows_,cols_>,rows_,cols_> {
public:
  typedef T value_type;

  OpenFDM_FORCE_INLINE
  Zeros(void)
  { }
  OpenFDM_FORCE_INLINE
  Zeros(size_type i, size_type j)
  { }
  OpenFDM_FORCE_INLINE
  ~Zeros(void)
  { }

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
  { return static_cast<value_type>(0); }

  OpenFDM_FORCE_INLINE
  Zeros<T,0,0>
  operator()(const Range& r1, const Range& r2 = Range(1)) const
  { return Zeros<T,0,0>(r1.last - r1.first + 1, r2.last - r2.first + 1); }
};

// A generic all zero matrix.
//  T is the value type for that one.
//  S is the size template.
template<typename, size_type, size_type>
class Eye;

// A dynamic sized matrix container.
//  T is the value type for that one.
template<typename T>
class Eye<T,0,0>
  : public MatrixRValue<Eye<T,0,0>,0,0> {
public:
  typedef T value_type;

  OpenFDM_FORCE_INLINE
  Eye(void)
    : rows_(0), cols_(0)
  { }
  OpenFDM_FORCE_INLINE
  Eye(size_type i, size_type j)
    : rows_(0), cols_(0)
  { resize(i, j); }
  OpenFDM_FORCE_INLINE
  Eye(const Eye& mtrx)
    : rows_(0), cols_(0)
  { resize(mtrx.rows(), mtrx.cols()); }
  OpenFDM_FORCE_INLINE
  ~Eye(void)
  { }

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
  { return i == j ? static_cast<value_type>(1) : static_cast<value_type>(0); }

  /** Resize to a given size.
   */
  OpenFDM_FORCE_INLINE
  bool resize(size_type i, size_type j)
  { rows_ = i; cols_ = j; return true; }
  template<typename Impl2, size_type m2, size_type n2>
  OpenFDM_FORCE_INLINE
  bool resize(const MatrixRValue<Impl2,m2,n2>& A)
  { return resize(A.asImpl().rows(), A.asImpl().cols()); }

  OpenFDM_FORCE_INLINE
  Eye& operator=(const Eye& mtrx)
  { resize(mtrx.rows(), mtrx.cols()); return *this; }

private:
  size_type rows_;
  size_type cols_;
};


// A dynamic sized matrix container.
//  T is the value type for that one.
template<typename T, size_type rows_, size_type cols_ >
class Eye
  : public MatrixRValue<Eye<T,rows_,cols_>,rows_,cols_> {
public:
  typedef T value_type;

  OpenFDM_FORCE_INLINE
  Eye(void)
  { }
  OpenFDM_FORCE_INLINE
  Eye(size_type i, size_type j)
  { }
  OpenFDM_FORCE_INLINE
  ~Eye(void)
  { }

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
  { return i == j ? static_cast<value_type>(1) : static_cast<value_type>(0); }
};

template<typename T>
class Vector2
  : public Vector<T,2> {
public:
  OpenFDM_FORCE_INLINE
  Vector2(void)
  { }
  OpenFDM_FORCE_INLINE
  Vector2(T v1, T v2)
  { (*this)(1) = v1; (*this)(2) = v2; }
  OpenFDM_FORCE_INLINE
  Vector2(const Vector2& v)
    : Vector<T,2>(v)
  { }
  template<typename Impl, size_type m>
  OpenFDM_FORCE_INLINE
  Vector2(const MatrixRValue<Impl,m,1>& v)
    : Vector<T,2>(v)
  { }
  OpenFDM_FORCE_INLINE
  ~Vector2(void)
  { }
};

template<typename T>
class Vector3
  : public Vector<T,3> {
public:
  OpenFDM_FORCE_INLINE
  Vector3(void)
  { }
  OpenFDM_FORCE_INLINE
  Vector3(T v1, T v2, T v3)
  { (*this)(1) = v1; (*this)(2) = v2; (*this)(3) = v3; }
  OpenFDM_FORCE_INLINE
  Vector3(const Vector3& v)
    : Vector<T,3>(v)
  { }
  template<typename Impl, size_type m>
  OpenFDM_FORCE_INLINE
  Vector3(const MatrixRValue<Impl,m,1>& v)
    : Vector<T,3>(v)
  { }
  OpenFDM_FORCE_INLINE
  ~Vector3(void)
  { }
};

template<typename T>
class Vector4
  : public Vector<T,4> {
public:
  OpenFDM_FORCE_INLINE
  Vector4(void)
  { }
  OpenFDM_FORCE_INLINE
  Vector4(T v1, T v2, T v3, T v4)
  { (*this)(1) = v1; (*this)(2) = v2; (*this)(3) = v3; (*this)(4) = v4; }
  OpenFDM_FORCE_INLINE
  Vector4(const Vector4& v)
    : Vector<T,4>(v)
  { }
  template<typename Impl, size_type m>
  OpenFDM_FORCE_INLINE
  Vector4(const MatrixRValue<Impl,m,1>& v)
    : Vector<T,4>(v)
  { }
  OpenFDM_FORCE_INLINE
  ~Vector4(void)
  { }
};

template<typename T>
class Vector6
  : public Vector<T,6> {
public:
  OpenFDM_FORCE_INLINE
  Vector6(void)
  { }
  OpenFDM_FORCE_INLINE
  Vector6(T v1, T v2, T v3, T v4, T v5, T v6)
  {
    (*this)(1) = v1; (*this)(2) = v2; (*this)(3) = v3;
    (*this)(4) = v4; (*this)(5) = v5; (*this)(6) = v6;
  }
  OpenFDM_FORCE_INLINE
  Vector6(const Vector3<T>& v1, const Vector3<T>& v2)
  {
    (*this)(1) = v1(1); (*this)(2) = v1(2); (*this)(3) = v1(3);
    (*this)(4) = v2(1); (*this)(5) = v2(2); (*this)(6) = v2(3);
  }
  OpenFDM_FORCE_INLINE
  Vector6(const Vector6& v)
    : Vector<T,6>(v)
  { }
  template<typename Impl, size_type m>
  OpenFDM_FORCE_INLINE
  Vector6(const MatrixRValue<Impl,m,1>& v)
    : Vector<T,6>(v)
  { }
  OpenFDM_FORCE_INLINE
  ~Vector6(void)
  { }

  OpenFDM_FORCE_INLINE
  Vector3<T> getAngular(void) const
  { return Vector3<T>((*this)(1), (*this)(2), (*this)(3)); }
  OpenFDM_FORCE_INLINE
  void setAngular(const Vector3<T>& v)
  { (*this)(1) = v(1); (*this)(2) = v(2); (*this)(3) = v(3); }

  OpenFDM_FORCE_INLINE
  Vector3<T> getLinear(void) const
  { return Vector3<T>((*this)(4), (*this)(5), (*this)(6)); }
  OpenFDM_FORCE_INLINE
  void setLinear(const Vector3<T>& v)
  { (*this)(4) = v(1); (*this)(5) = v(2); (*this)(6) = v(3); }
};

template<typename T>
class Matrix22
  : public Matrix<T,2,2> {
public:
  OpenFDM_FORCE_INLINE
  Matrix22(void)
  { }
  OpenFDM_FORCE_INLINE
  Matrix22(T m11, T m12, T m21, T m22)
  {
    (*this)(1, 1) = m11; (*this)(1, 2) = m12;
    (*this)(2, 1) = m21; (*this)(2, 2) = m22;
  }
  OpenFDM_FORCE_INLINE
  Matrix22(const Matrix22& m)
    : Matrix<T,2,2>(m)
  { }
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Matrix22(const MatrixRValue<Impl,2,2>& A)
    : Matrix<T,2,2>(A)
  { }
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Matrix22(const MatrixRValue<Impl,2,0>& A)
    : Matrix<T,2,2>(A)
  { }
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Matrix22(const MatrixRValue<Impl,0,2>& A)
    : Matrix<T,2,2>(A)
  { }
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Matrix22(const MatrixRValue<Impl,0,0>& A)
    : Matrix<T,2,2>(A)
  { }
  OpenFDM_FORCE_INLINE
  ~Matrix22(void)
  { }
};

template<typename T>
class Matrix33
  : public Matrix<T,3,3> {
public:
  OpenFDM_FORCE_INLINE
  Matrix33(void)
  { }
  OpenFDM_FORCE_INLINE
  Matrix33(T m11, T m12, T m13, T m21, T m22, T m23, T m31, T m32, T m33)
  {
    (*this)(1, 1) = m11; (*this)(1, 2) = m12; (*this)(1, 3) = m13;
    (*this)(2, 1) = m21; (*this)(2, 2) = m22; (*this)(2, 3) = m23;
    (*this)(3, 1) = m31; (*this)(3, 2) = m32; (*this)(3, 3) = m33;
  }
  OpenFDM_FORCE_INLINE
  Matrix33(const Matrix33& m)
    : Matrix<T,3,3>(m)
  { }
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Matrix33(const MatrixRValue<Impl,3,3>& A)
    : Matrix<T,3,3>(A)
  { }
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Matrix33(const MatrixRValue<Impl,3,0>& A)
    : Matrix<T,3,3>(A)
  { }
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Matrix33(const MatrixRValue<Impl,0,3>& A)
    : Matrix<T,3,3>(A)
  { }
  template<typename Impl>
  OpenFDM_FORCE_INLINE
  Matrix33(const MatrixRValue<Impl,0,0>& A)
    : Matrix<T,3,3>(A)
  { }
  OpenFDM_FORCE_INLINE
  ~Matrix33(void)
  { }
};

template<typename T>
class SymMatrix3
  : public SymMatrix<T,3> {
public:
  OpenFDM_FORCE_INLINE
  SymMatrix3(void)
  { }
  OpenFDM_FORCE_INLINE
  SymMatrix3(T S11, T S21, T S31, T S22, T S32, T S33)
  {
    (*this)(1, 1) = S11;
    (*this)(2, 1) = S21; (*this)(2, 2) = S22;
    (*this)(3, 1) = S31; (*this)(3, 2) = S32; (*this)(3, 3) = S33;
  }
  OpenFDM_FORCE_INLINE
  SymMatrix3(const SymMatrix<T,3>& S)
    : SymMatrix<T,3>(S)
  { }
  OpenFDM_FORCE_INLINE
  SymMatrix3(const SymMatrix3& S)
    : SymMatrix<T,3>(S)
  { }
  OpenFDM_FORCE_INLINE
  ~SymMatrix3(void)
  { }
};

template<typename T>
class SymMatrix6
  : public SymMatrix<T,6> {
public:
  OpenFDM_FORCE_INLINE
  SymMatrix6(void)
  { }
  OpenFDM_FORCE_INLINE
  SymMatrix6(T m)
  {
    (*this)(1,1) = 0;
    (*this)(2,1) = 0; (*this)(2,2) = 0;
    (*this)(3,1) = 0; (*this)(3,2) = 0; (*this)(3,3) = 0;
    (*this)(4,1) = 0; (*this)(4,2) = 0; (*this)(4,3) = 0;
    (*this)(4,4) = m;
    (*this)(5,1) = 0; (*this)(5,2) = 0; (*this)(5,3) = 0;
    (*this)(5,4) = 0; (*this)(5,5) = m;
    (*this)(6,1) = 0; (*this)(6,2) = 0; (*this)(6,3) = 0;
    (*this)(6,4) = 0; (*this)(6,5) = 0; (*this)(6,6) = m;
  }
  OpenFDM_FORCE_INLINE
  SymMatrix6(const SymMatrix<T,3>& I, T m)
  {
    (*this)(1,1) = I(1,1);
    (*this)(2,1) = I(2,1); (*this)(2,2) = I(2,2);
    (*this)(3,1) = I(3,1); (*this)(3,2) = I(3,2); (*this)(3,3) = I(3,3);
    (*this)(4,1) = 0;      (*this)(4,2) = 0;      (*this)(4,3) = 0;
    (*this)(4,4) = m;
    (*this)(5,1) = 0;      (*this)(5,2) = 0;      (*this)(5,3) = 0;
    (*this)(5,4) = 0;      (*this)(5,5) = m;
    (*this)(6,1) = 0;      (*this)(6,2) = 0;      (*this)(6,3) = 0;
    (*this)(6,4) = 0;      (*this)(6,5) = 0;      (*this)(6,6) = m;
  }
  OpenFDM_FORCE_INLINE
  SymMatrix6(T S11,
             T S21, T S22,
             T S31, T S32, T S33,
             T S41, T S42, T S43, T S44,
             T S51, T S52, T S53, T S54, T S55,
             T S61, T S62, T S63, T S64, T S65, T S66)
  {
    (*this)(1,1) = S11;
    (*this)(2,1) = S21; (*this)(2,2) = S22;
    (*this)(3,1) = S31; (*this)(3,2) = S32; (*this)(3,3) = S33;
    (*this)(4,1) = S41; (*this)(4,2) = S42; (*this)(4,3) = S43;
    (*this)(4,4) = S44;
    (*this)(5,1) = S51; (*this)(5,2) = S52; (*this)(5,3) = S53;
    (*this)(5,4) = S54; (*this)(5,5) = S55;
    (*this)(6,1) = S61; (*this)(6,2) = S62; (*this)(6,3) = S63;
    (*this)(6,4) = S64; (*this)(6,5) = S65; (*this)(6,6) = S66;
  }
  OpenFDM_FORCE_INLINE
  SymMatrix6(const SymMatrix<T,6>& S)
    : SymMatrix<T,6>(S)
  { }
  OpenFDM_FORCE_INLINE
  SymMatrix6(const SymMatrix6& S)
    : SymMatrix<T,6>(S)
  { }
  OpenFDM_FORCE_INLINE
  ~SymMatrix6(void)
  { }

  OpenFDM_FORCE_INLINE
  static SymMatrix6 zeros(void)
  { return SymMatrix6(Zeros<T,6,6>(6,6)); }

};

} // namespace LinAlg

} // namespace OpenFDM

#endif
