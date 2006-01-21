/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich
 *
 */

#ifndef OpenFDM_TableData_H
#define OpenFDM_TableData_H

#include <iosfwd>
#include <map>
#include "Vector.h"

namespace OpenFDM {

class TableLookup {
  typedef std::map<real_type, unsigned> Table;
  typedef std::pair<real_type, unsigned> Pair;
 
public:
  TableLookup(void)
  {}

  /// Return the breakpoint at the given index i
  /// If it does not exist in the lookup table, zero is returned
  real_type getAtIndex(unsigned i) const
  {
    Table::const_iterator it;
    for (it = mTable.begin(); it != mTable.end(); ++it) {
      if (it->second == i)
        return it->first;
    }
    return 0;
  }
  /// Set the breakpoint value at index i, the lookup table is extended if
  /// required. Keep in mind, that the table lookup behaves undefined if
  /// the sequence of indices does not increase or decrease strictly monotonic
  /// with the values.
  void setAtIndex(unsigned i, real_type value)
  {
    Table::iterator it = mTable.begin();
    while (it != mTable.end()) {
      if (it->second == i) {
        mTable.erase(it);
        it = mTable.begin();
      }
      else
        ++it;
    }
    mTable.insert(it, Pair(value, i));
  }

  /// Returns the size of the lookup table
  unsigned size(void) const
  {
    unsigned sz = 0;
    Table::const_iterator it;
    for (it = mTable.begin(); it != mTable.end(); ++it) {
      sz = max(sz, it->second);
    }
    return sz;
  }

  /// Check for consistency, that is, the breakpoint indices are strictly
  /// ordered and there are no holes in the sequence of indices
  bool isValid(void) const
  {
    Table::const_iterator it = mTable.begin();
    if (it == mTable.end())
      return false;
    int indexDir = 0;
    int valueDir = 0;
    real_type prevValue = it->first;
    unsigned prevIndex = it->second;
    for (++it; it != mTable.end(); ++it) {
      // We do not yet know which direction we should check
      if (indexDir == 0) {
        // Check for the direction of the indices
        if (prevIndex + 1 == it->second) {
          indexDir = 1;
        } else if (prevIndex == it->second + 1) {
          indexDir = -1;
        } else {
          // Duplicate index ...
          return false;
        }
        
        // Check for the direction of the lookup keys
        if (prevValue < it->first) {
          valueDir = 1;
        } else if (prevValue > it->first) {
          valueDir = -1;
        } else {
          // Duplicate lookup keys ...
          return false;
        }

      } else if (indexDir == -1) {
        // Check if the direction is still the same
        if (prevIndex != it->second + 1)
          return false;
        if (prevValue*valueDir >= it->first*valueDir)
          return false;
      } else {
        // Check if the direction is still the same
        if (prevIndex + 1 != it->second)
          return false;
        if (prevValue*valueDir >= it->first*valueDir)
          return false;
      }

      prevValue = it->first;
      prevIndex = it->second;
    }
    return true;
  }

  real_type lookup(real_type input) const
  {
    // Empty table??
    // FIXME
    if (mTable.empty())
      return 1;

    // Find the table bounds for the requested input.
    Table::const_iterator upBoundIt = mTable.upper_bound(input);
    Table::const_iterator loBoundIt = upBoundIt;
    --loBoundIt;

    Table::const_iterator beg = mTable.begin();
    if (upBoundIt == beg)
      return 1;
    if (upBoundIt == mTable.end()) {
      unsigned last = mTable.rbegin()->second;
      return last;
    }

    // Just do linear interpolation.
    real_type loBound = loBoundIt->first;
    real_type upBound = upBoundIt->first;
    unsigned loIdx = loBoundIt->second;
    if (loBound == upBound)
      return loIdx;
    unsigned upIdx = upBoundIt->second;
    real_type theta = (input - loBound)/(upBound-loBound);
    return loIdx + theta;
  }

  bool operator==(const TableLookup& tl) const
  {
    mTable == tl.mTable;
  }

private:
  Table mTable;
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
    mSize(size), mData(new real_type[product(size)])
  {
    unsigned totalLen = product(mSize);
    for (unsigned i = 0; i < totalLen; ++i)
      mData[i] = 0;
  }
  TableData(const TableData& ndarray) :
    mSize(ndarray.size()), mData(new real_type[product(ndarray.size())])
  {
    unsigned totalLen = product(mSize);
    for (unsigned i = 0; i < totalLen; ++i)
      mData[i] = ndarray.mData[i];
  }
  ~TableData(void)
  { delete[] mData; }

  TableData& operator=(const TableData& ndarray)
  {
    delete[] mData;
    if (ndarray.mData) {
      mSize = ndarray.size();
      unsigned totalLen = product(mSize);
      mData = new real_type[totalLen];
      for (unsigned i = 0; i < totalLen; ++i)
        mData[i] = ndarray.mData[i];
    } else {
      mData = 0;
    }
    return *this;
  }

  const SizeVector& size(void) const
  { return mSize; }
  unsigned size(unsigned i) const
  { if (i < 1 || mSize.size() < i) return 0; return mSize(i); }

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
    unsigned idx = multiIndex(numDims) - 1;
    for (unsigned i = numDims-1; 0 < i; --i) {
      idx *= mSize(i);
      idx += multiIndex(i) - 1;
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

    real_type ridx = interp(indexNum);

    // Check for an out of range index
    // Note that this negated check also catches NaNs
    if (!(1 <= ridx)) {
      curIndex(indexNum) = 1;
      return interpolator(indexNum-1, curIndex, interp);
    }

    // Check for an out of range index
    unsigned sz = mSize(indexNum);
    if (sz < ridx) {
      curIndex(indexNum) = sz;
      return interpolator(indexNum-1, curIndex, interp);
    }

    unsigned i0 = unsigned(floor(ridx));
    unsigned i1 = unsigned(ceil(ridx));
    if (i0 == i1) {
      // Exactly hit an integer valued index
      curIndex(indexNum) = i0;
      return interpolator(indexNum-1, curIndex, interp);
    }

    // Need interpolation in this dimension
    curIndex(indexNum) = i0;
    real_type value;
    value = (i1 - ridx) * interpolator(indexNum-1, curIndex, interp);
    
    curIndex(indexNum) = i1;
    value += (ridx - i0) * interpolator(indexNum-1, curIndex, interp);
    
    return value;
  }

  real_type* mData;
  SizeVector mSize;
};

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os, const TableLookup& tl)
{
  for (unsigned idx = 1; idx <= tl.size(); ++idx) {
    os << tl.getAtIndex(idx) << ' ';
  }
  return os;
}

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os, const TableData<1>& td)
{
  TableData<1>::SizeVector sz = td.size();

  for (unsigned idx = 1; idx <= sz(1); ++idx) {
    TableData<1>::Index multiIdx;
    multiIdx(1) = idx;
    os << td(multiIdx) << ' ';
  }
  return os;
}

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os, const TableData<2>& td)
{
  TableData<2>::SizeVector sz = td.size();

  for (unsigned i = 1; i <= sz(1); ++i) {
    for (unsigned j = 1; j <= sz(2); ++j) {
      TableData<2>::Index multiIdx;
      multiIdx(1) = i;
      multiIdx(2) = j;
      os << td(multiIdx) << ' ';
    }
    os << '\n';
  }
  return os;
}

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os, const TableData<3>& td)
{
  TableData<3>::SizeVector sz = td.size();

  for (unsigned i = 1; i <= sz(1); ++i) {
    for (unsigned j = 1; j <= sz(2); ++j) {
      for (unsigned k = 1; k <= sz(3); ++k) {
        TableData<3>::Index multiIdx;
        multiIdx(1) = i;
        multiIdx(2) = j;
        multiIdx(3) = k;
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
