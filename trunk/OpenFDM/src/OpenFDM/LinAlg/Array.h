/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LinAlgArray_H
#define OpenFDM_LinAlgArray_H

namespace OpenFDM {

namespace LinAlg {

template<typename T, size_type size_>
class LinearArray;

template<typename T, size_type size_>
class LinearArray {
public:
  typedef T                 value_type;
  
  OpenFDM_FORCE_INLINE
  LinearArray(void)
  { }
  OpenFDM_FORCE_INLINE
  LinearArray(size_type size)
  { OpenFDMLinAlgAssert(size == size_); }
   
  /// Element access.
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i)
  { return data_ + index(i); }
  /// Const element access.
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i) const
  { return data_ + index(i); }

  OpenFDM_FORCE_INLINE
  size_type index(size_type i) const
  { OpenFDMLinAlgAssert(i < size()); return i; }

  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  size_type size() const
  { return size_; }
  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  bool resize(size_type size)
  { OpenFDMLinAlgAssert(size == size_); return size == size_; }
  
private:
  LinearArray(const LinearArray& va);
  LinearArray& operator=(const LinearArray& va);

  value_type data_[size_] OpenFDM_Align_SSE2Double;
};

template<typename T>
class LinearArray<T,0> {
public:
  typedef T                 value_type;
  
  OpenFDM_FORCE_INLINE
  LinearArray(void)
    : data_(0), size_(0)
  { }
  OpenFDM_FORCE_INLINE
  LinearArray(size_type size)
    : data_(0), size_(0)
  { resize(size); }

  OpenFDM_FORCE_INLINE
  ~LinearArray()
  { delete[] data_; }

  /// Element access.
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i)
  { return data_ + index(i); }
  /// Const element access.
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i) const
  { return data_ + index(i); }

  OpenFDM_FORCE_INLINE
  size_type index(size_type i) const
  { OpenFDMLinAlgAssert(i < size()); return i; }

  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  size_type size() const
  { return size_; }
  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  bool resize(size_type size)
  {
    if (size == size_)
      return true;
    delete[] data_;
    size_ = 0;
    data_ = 0;
    data_ = new value_type[size];
    if (!data_)
      return false;
    size_ = size;
    return true;
  }
  
private:
  LinearArray(const LinearArray& va);
  LinearArray& operator=(const LinearArray& va);

  value_type* data_;
  size_type size_;
};

template<typename T, size_type rows_, size_type cols_>
class RectangularArray;

template<typename T, size_type rows_, size_type cols_>
class RectangularArray {
public:
  typedef T                 value_type;
  
  OpenFDM_FORCE_INLINE
  RectangularArray(void)
  { }
  OpenFDM_FORCE_INLINE
  RectangularArray(size_type rows, size_type cols)
  { resize(rows, cols); }
   
  /// Element access.
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_ + index(i, j); }
  /// Const element access.
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_ + index(i, j); }

  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  {
    OpenFDMLinAlgAssert(i < rows());
    OpenFDMLinAlgAssert(j < cols());
    return i + j*rows_;
  }

  OpenFDM_FORCE_INLINE
  size_type rows() const
  { return rows_; }
  OpenFDM_FORCE_INLINE
  size_type cols() const
  { return cols_; }
  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  bool resize(size_type rows, size_type cols)
  {
    OpenFDMLinAlgAssert(rows == rows_);
    OpenFDMLinAlgAssert(cols == cols_);
    return rows == rows_ && cols == cols_;
  }
  
private:
  RectangularArray(const RectangularArray& va);
  RectangularArray& operator=(const RectangularArray& va);

  value_type data_[rows_*cols_] OpenFDM_Align_SSE2Double;
};

template<typename T, size_type cols_>
class RectangularArray<T,0,cols_> {
public:
  typedef T                 value_type;
  
  OpenFDM_FORCE_INLINE
  RectangularArray(void)
    : data_(0), rows_(0)
  { }
  OpenFDM_FORCE_INLINE
  RectangularArray(size_type rows, size_type cols)
    : data_(0), rows_(0)
  { resize(rows, cols); }

  OpenFDM_FORCE_INLINE
  ~RectangularArray()
  { delete[] data_; }

  /// Element access.
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_ + index(i, j); }
  /// Const element access.
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_ + index(i, j); }

  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  {
    OpenFDMLinAlgAssert(i < rows());
    OpenFDMLinAlgAssert(j < cols());
    return i + j*rows_;
  }

  OpenFDM_FORCE_INLINE
  size_type rows() const
  { return rows_; }
  OpenFDM_FORCE_INLINE
  size_type cols() const
  { return cols_; }
  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  bool resize(size_type rows, size_type cols)
  {
    OpenFDMLinAlgAssert(cols == cols_);
    if (rows == rows_ && cols == cols_)
      return true;
    
    delete[] data_;
    rows_ = 0;
    data_ = new value_type[rows*cols];
    if (!data_)
      return false;
    rows_ = rows;
    return true;
  }
  
private:
  RectangularArray(const RectangularArray& va);
  RectangularArray& operator=(const RectangularArray& va);

  value_type* data_;
  size_type rows_;
};

template<typename T, size_type rows_>
class RectangularArray<T,rows_,0> {
public:
  typedef T                 value_type;
  
  OpenFDM_FORCE_INLINE
  RectangularArray(void)
    : data_(0), cols_(0)
  { }
  OpenFDM_FORCE_INLINE
  RectangularArray(size_type rows, size_type cols)
    : data_(0), cols_(0)
  { resize(rows, cols); }

  OpenFDM_FORCE_INLINE
  ~RectangularArray()
  { delete[] data_; }

  /// Element access.
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_ + index(i, j); }
  /// Const element access.
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_ + index(i, j); }

  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  {
    OpenFDMLinAlgAssert(i < rows());
    OpenFDMLinAlgAssert(j < cols());
    return i + j*rows_;
  }

  OpenFDM_FORCE_INLINE
  size_type rows() const
  { return rows_; }
  OpenFDM_FORCE_INLINE
  size_type cols() const
  { return cols_; }
  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  bool resize(size_type rows, size_type cols)
  {
    OpenFDMLinAlgAssert(rows == rows_);
    if (rows == rows_ && cols == cols_)
      return true;
    
    delete[] data_;
    cols_ = 0;
    data_ = new value_type[rows*cols];
    if (!data_)
      return false;
    cols_ = cols;
    return true;
  }
  
private:
  RectangularArray(const RectangularArray& va);
  RectangularArray& operator=(const RectangularArray& va);

  value_type* data_;
  size_type cols_;
};

template<typename T>
class RectangularArray<T,0,0> {
public:
  typedef T                 value_type;
  
  OpenFDM_FORCE_INLINE
  RectangularArray(void)
    : data_(0), rows_(0), cols_(0)
  { }
  OpenFDM_FORCE_INLINE
  RectangularArray(size_type rows, size_type cols)
    : data_(0), rows_(0), cols_(0)
  { resize(rows, cols); }

  OpenFDM_FORCE_INLINE
  ~RectangularArray()
  { delete[] data_; }

  /// Element access.
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_ + index(i, j); }
  /// Const element access.
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_ + index(i, j); }

  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  {
    OpenFDMLinAlgAssert(i < rows());
    OpenFDMLinAlgAssert(j < cols());
    return i + j*rows_;
  }

  OpenFDM_FORCE_INLINE
  size_type rows() const
  { return rows_; }
  OpenFDM_FORCE_INLINE
  size_type cols() const
  { return cols_; }
  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  bool resize(size_type rows, size_type cols)
  {
    if (rows == rows_ && cols == cols_)
      return true;
    
    delete[] data_;
    rows_ = 0;
    cols_ = 0;
    data_ = new value_type[rows*cols];
    if (!data_)
      return false;
    rows_ = rows;
    cols_ = cols;
    return true;
  }
  
private:
  RectangularArray(const RectangularArray& va);
  RectangularArray& operator=(const RectangularArray& va);

  value_type* data_;
  size_type rows_;
  size_type cols_;
};

template<typename T, size_type size_>
class SymmetricArray;

template<typename T, size_type size_>
class SymmetricArray {
public:
  typedef T                 value_type;
  
  OpenFDM_FORCE_INLINE
  SymmetricArray(void)
  { }
  OpenFDM_FORCE_INLINE
  SymmetricArray(size_type size)
  { OpenFDMLinAlgAssert(size == size_); }
   
  /// Element access.
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_ + index(i, j); }
  /// Const element access.
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_ + index(i, j); }

  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  {
    // For details see Golumb/Van Loan: Matrix Computations, 20pp.
    OpenFDMLinAlgAssert(i < rows());
    OpenFDMLinAlgAssert(j < cols());
    if (i >= j)
      return j*size_ - (j*(j+1))/2 + i;
    else
      return i*size_ - (i*(i+1))/2 + j;
  }

  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  size_type size() const
  { return size_; }
  OpenFDM_FORCE_INLINE
  size_type rows() const
  { return size_; }
  OpenFDM_FORCE_INLINE
  size_type cols() const
  { return size_; }
  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  bool resize(size_type size)
  { OpenFDMLinAlgAssert(size == size_); return size == size_; }
  
private:
  SymmetricArray(const SymmetricArray& va);
  SymmetricArray& operator=(const SymmetricArray& va);

  value_type data_[(size_*(size_+1))/2];
};

template<typename T>
class SymmetricArray<T,0> {
public:
  typedef T                 value_type;
  
  OpenFDM_FORCE_INLINE
  SymmetricArray(void)
    : data_(0), size_(0)
  { }
  OpenFDM_FORCE_INLINE
  SymmetricArray(size_type size)
    : data_(0), size_(0)
  { resize(size); }

  OpenFDM_FORCE_INLINE
  ~SymmetricArray()
  { delete[] data_; }

  /// Element access.
  OpenFDM_FORCE_INLINE
  value_type* find(size_type i, size_type j)
  { return data_ + index(i, j); }
  /// Const element access.
  OpenFDM_FORCE_INLINE
  const value_type* find(size_type i, size_type j) const
  { return data_ + index(i, j); }

  OpenFDM_FORCE_INLINE
  size_type index(size_type i, size_type j) const
  {
    // For details see Golumb/Van Loan: Matrix Computations, 20pp.
    OpenFDMLinAlgAssert(i < rows());
    OpenFDMLinAlgAssert(j < cols());
    if (i >= j)
      return j*size_ - (j*(j+1))/2 + i;
    else
      return i*size_ - (i*(i+1))/2 + j;
  }

  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  size_type size() const
  { return size_; }
  OpenFDM_FORCE_INLINE
  size_type rows() const
  { return size_; }
  OpenFDM_FORCE_INLINE
  size_type cols() const
  { return size_; }
  /// Size of the vector.
  OpenFDM_FORCE_INLINE
  bool resize(size_type size)
  {
    if (size == size_)
      return true;
    delete[] data_;
    size_ = 0;
    data_ = 0;
    data_ = new value_type[(size*(size+1))/2];
    if (!data_)
      return false;
    size_ = size;
    return true;
  }
  
private:
  SymmetricArray(const SymmetricArray& va);
  SymmetricArray& operator=(const SymmetricArray& va);

  value_type* data_;
  size_type size_;
};

} // namespace LinAlg

} // namespace OpenFDM

#endif
