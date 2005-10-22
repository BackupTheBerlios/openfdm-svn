/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich
 *
 */

#ifndef OpenFDM_Table_H
#define OpenFDM_Table_H

#include <map>
#include "Matrix.h"
#include "Vector.h"

#include "Model.h"

namespace OpenFDM {

struct InterploationData {
  InterploationData(void) {}
  InterploationData(unsigned index0, unsigned index1, real_type _theta)
  {
    theta[0] = _theta;
    theta[1] = 1 - _theta;
    index[0] = index0;
    index[1] = index1;
  }
  real_type theta[2];
  unsigned index[2];
};

class TableLookup {
private:
  /// FIXME may be use a std::vector for that.
  typedef std::map<real_type,unsigned> Table;

public:
  TableLookup(void)
  {}

  unsigned size(void) const
  { return mTable.size(); }

  void addBreakPoint(real_type value)
  {
    mTable[value] = 0;
    unsigned index = 1;
    Table::iterator it = mTable.begin();
    while (it != mTable.end()) {
      it->second = index;
      ++index;
      ++it;
    }
  }

  InterploationData lookup(real_type input) const
  {
    // Empty table??
    // FIXME
    if (mTable.empty())
      return InterploationData(1, 1, 0);

    // Find the table bounds for the requested input.
    Table::const_iterator upBoundIt = mTable.upper_bound(input);
    Table::const_iterator loBoundIt = upBoundIt;
    --loBoundIt;

    Table::const_iterator beg = mTable.begin();
    if (upBoundIt == beg)
      return InterploationData(1, 1, 0);
    if (upBoundIt == mTable.end()) {
      unsigned last = mTable.rbegin()->second;
      return InterploationData(last, last, 0);
    }

    // Just do linear interpolation.
    real_type loBound = loBoundIt->first;
    real_type upBound = upBoundIt->first;
    unsigned loIdx = loBoundIt->second;
    if (loBound == upBound)
      return InterploationData(loIdx, loIdx, 0);
    unsigned upIdx = upBoundIt->second;
    real_type theta = (input - loBound)/(upBound-loBound);
    return InterploationData(loIdx, upIdx, theta);
  }

private:
  Table mTable;
};

template<unsigned numDims>
class TableData {
public:
  typedef LinAlg::Vector<unsigned,numDims>  Index;
  typedef LinAlg::Vector<unsigned,numDims>  SizeVector;
  typedef LinAlg::Vector<InterploationData,numDims> InterpVector;

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

    real_type value = 0;

    curIndex(indexNum) = interp(indexNum).index[0];
    value += interp(indexNum).theta[0] *
      interpolator(indexNum-1, curIndex, interp);

    curIndex(indexNum) = interp(indexNum).index[1];
    value += interp(indexNum).theta[1] *
      interpolator(indexNum-1, curIndex, interp);

    return value;
  }

  real_type* mData;
  SizeVector mSize;
};

class Table1D : public Model {
public:
  Table1D(const std::string& name) :
    Model(name)
  {
    setDirectFeedThrough(true);
    
    setNumInputPorts(1);
    setInputPortName(0, "input");
    
    setNumOutputPorts(1);
    setOutputPort(0, "output", Property(this, &Table1D::getOutput));
    addProperty("output", Property(this, &Table1D::getOutput));
  }
  virtual ~Table1D(void) {}
  
  virtual bool init(void)
  {
    OpenFDMAssert(getInputPort(0).isValid());
  
    return getInputPort(0).isValid();
  }

  virtual void output(const TaskInfo&)
  {
    OpenFDMAssert(getInputPort(0).isValid());
    TableData<1>::InterpVector interpVec;
    real_type iv = getInputPort(0).getValue().toReal();
    interpVec(1) = mTableLookup.lookup(iv);
    mOutput = mTableData.interpolate(interpVec);
  }

  const real_type& getOutput(void) const
  { return mOutput; }

  void setTable(const TableData<1>& table)
  { mTableData = table; }
  void setTableData(const TableData<1>& table)
  { mTableData = table; }

  const TableData<1>& getTable(void) const 
  { return mTableData; }
  const TableData<1>& getTableData(void) const 
  { return mTableData; }
  TableData<1>& getTable(void)
  { return mTableData; }
  TableData<1>& getTableData(void)
  { return mTableData; }

  void setTableLookup(const TableLookup& tl)
  {
    mTableLookup = tl;
  }

private:
  real_type mOutput;
  TableData<1> mTableData;
  TableLookup mTableLookup;
};

} // namespace OpenFDM

#endif
