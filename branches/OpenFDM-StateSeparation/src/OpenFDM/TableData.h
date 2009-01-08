/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich
 *
 */

#ifndef OpenFDM_TableData_H
#define OpenFDM_TableData_H

#include <iosfwd>
#include <map>
#include <vector>
#include <algorithm>
#include "Vector.h"

namespace OpenFDM {

class BreakPointVector {
public:
  typedef real_type value_type;
  typedef std::vector<value_type> vector_type;

  /// Return the breakpoint at the given index i
  /// If it does not exist in the lookup table, zero is returned
  const value_type& operator[](unsigned i) const
  { OpenFDMAssert(i < mVector.size()); return mVector[i]; }

  /// Set the breakpoint value at index i, the lookup table is extended if
  /// required. Keep in mind, that the table lookup behaves undefined if
  /// the sequence of indices does not increase or decrease strictly monotonic
  /// with the values.
  void insert(const value_type& value)
  {
    vector_type::iterator i;
    i = std::lower_bound(mVector.begin(), mVector.end(), value);
    mVector.insert(i, value);
    mVector.erase(std::unique(mVector.begin(), mVector.end()), mVector.end());
  }

  /// Returns the size of the lookup table
  unsigned size(void) const
  { return mVector.size(); }

  value_type lookup(const value_type& input) const
  {
    // Empty table??
    // FIXME
    if (mVector.empty())
      return value_type(0);

    vector_type::const_iterator vectorBegin = mVector.begin();
    vector_type::const_iterator vectorEnd = mVector.end();

    // Find the table bounds for the requested input.
    vector_type::const_iterator upBoundIt;
    upBoundIt = std::upper_bound(vectorBegin, vectorEnd, input);
    vector_type::const_iterator loBoundIt = upBoundIt;
    --loBoundIt;

    if (upBoundIt == vectorBegin)
      return value_type(0);
    if (upBoundIt == vectorEnd)
      return value_type(mVector.size() - 1);

    // Just do linear interpolation.
    value_type loIdx = value_type(std::distance(vectorBegin, loBoundIt));
    if (loBoundIt == upBoundIt)
      return loIdx;
    return loIdx + (input - *loBoundIt)/(*upBoundIt - *loBoundIt);
  }

  bool operator==(const BreakPointVector& bv) const
  { return mVector == bv.mVector; }

private:
  vector_type mVector;
};

template<unsigned numDims>
class TableData {
public:
  typedef LinAlg::Vector<unsigned,numDims>  Index;
  typedef LinAlg::Vector<unsigned,numDims>  SizeVector;
  typedef LinAlg::Vector<real_type,numDims> InterpVector;

  TableData(void) :
    mData(0)
  {}
  TableData(const SizeVector& size) :
    mData(new real_type[product(size)]), mSize(size)
  {
    unsigned totalLen = product(mSize);
    for (unsigned i = 0; i < totalLen; ++i)
      mData[i] = 0;
  }
  TableData(const TableData& ndarray) :
    mData(new real_type[product(ndarray.size())]), mSize(ndarray.size())
  {
    unsigned totalLen = product(mSize);
    for (unsigned i = 0; i < totalLen; ++i)
      mData[i] = ndarray.mData[i];
  }
  ~TableData(void)
  { delete[] mData; }

  TableData& operator=(const TableData& ndarray)
  {
    if (this == &ndarray)
      return *this;
    delete[] mData;
    if (ndarray.mData) {
      mSize = ndarray.size();
      unsigned totalLen = product(mSize);
      if (0 < totalLen) {
        mData = new real_type[totalLen];
        for (unsigned i = 0; i < totalLen; ++i)
          mData[i] = ndarray.mData[i];
      } else {
        mData = 0;
      }
    } else {
      mData = 0;
    }
    return *this;
  }

  const SizeVector& size(void) const
  { return mSize; }
  unsigned size(unsigned i) const
  { if (mSize.size() <= i) return 0; return mSize(i); }
  //   { if (mSize.size() <= i) return 1; /*??may be*/ return mSize(i); }

  const real_type& operator()(const Index& multiIndex) const
  { return mData[offset(multiIndex)]; }
  real_type& operator()(const Index& multiIndex)
  { return mData[offset(multiIndex)]; }

  const real_type& at(const Index& multiIndex) const
  { return mData[offset(multiIndex)]; }
  real_type& at(const Index& multiIndex)
  { return mData[offset(multiIndex)]; }

  unsigned offset(const Index& multiIndex) const
  {
    /// FIXME do size bounds checking ...
    unsigned idx = multiIndex(numDims-1);
    for (unsigned i = numDims-1; 0 < i; --i) {
      idx *= mSize(i-1);
      idx += multiIndex(i-1);
    }
    return idx;
  }

  real_type interpolate(const InterpVector& interp) const
  {
    Index curIndex;
    return interpolator(numDims, curIndex, interp);
  }
private:
  real_type interpolator(unsigned indexNum, Index& curIndex,
                         const InterpVector& interp) const
  {
    if (indexNum == 0)
      return at(curIndex);

    real_type ridx = interp(indexNum-1);

    // Check for an out of range index
    // Note that this negated check also catches NaNs
    if (!(0 <= ridx)) {
      curIndex(indexNum-1) = 0;
      return interpolator(indexNum-1, curIndex, interp);
    }

    // Check for an out of range index
    unsigned sz = mSize(indexNum-1);
    if (sz <= ridx) {
      curIndex(indexNum-1) = sz-1;
      return interpolator(indexNum-1, curIndex, interp);
    }

    unsigned i0 = unsigned(floor(ridx));
    unsigned i1 = unsigned(ceil(ridx));
    if (i0 == i1) {
      // Exactly hit an integer valued index
      curIndex(indexNum-1) = i0;
      return interpolator(indexNum-1, curIndex, interp);
    }

    // Need interpolation in this dimension
    curIndex(indexNum-1) = i0;
    real_type value;
    value = (i1 - ridx) * interpolator(indexNum-1, curIndex, interp);
    
    curIndex(indexNum-1) = i1;
    value += (ridx - i0) * interpolator(indexNum-1, curIndex, interp);
    
    return value;
  }

  real_type* mData;
  SizeVector mSize;
};

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os,
           const BreakPointVector& breakPointVector)
{
  for (unsigned i = 0; i < breakPointVector.size(); ++i)
    os << breakPointVector[i] << ' ';
  return os;
}

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os,
           const TableData<1>& td)
{
  TableData<1>::SizeVector sz = td.size();

  for (unsigned idx = 0; idx < sz(0); ++idx) {
    TableData<1>::Index multiIdx;
    multiIdx(0) = idx;
    os << td(multiIdx) << ' ';
  }
  return os;
}

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os,
           const TableData<2>& td)
{
  TableData<2>::SizeVector sz = td.size();

  for (unsigned i = 0; i < sz(0); ++i) {
    for (unsigned j = 0; j < sz(1); ++j) {
      TableData<2>::Index multiIdx;
      multiIdx(0) = i;
      multiIdx(1) = j;
      os << td(multiIdx) << ' ';
    }
    os << '\n';
  }
  return os;
}

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os,
           const TableData<3>& td)
{
  TableData<3>::SizeVector sz = td.size();

  for (unsigned i = 0; i < sz(0); ++i) {
    for (unsigned j = 0; j < sz(1); ++j) {
      for (unsigned k = 0; k < sz(2); ++k) {
        TableData<3>::Index multiIdx;
        multiIdx(0) = i;
        multiIdx(1) = j;
        multiIdx(2) = k;
        os << td(multiIdx) << ' ';
      }
      os << '\n';
    }
    os << '\n';
  }
  return os;
}

} // namespace OpenFDM

#endif
