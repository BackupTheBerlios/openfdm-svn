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
    weight[0] = 1 - _theta;
    weight[1] = _theta;
    index[0] = index0;
    index[1] = index1;
  }
  real_type weight[2];
  unsigned index[2];
};

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
    value += interp(indexNum).weight[0] *
      interpolator(indexNum-1, curIndex, interp);

    curIndex(indexNum) = interp(indexNum).index[1];
    value += interp(indexNum).weight[1] *
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
    setOutputPort(0, "output", this, &Table1D::getOutput);

    addProperty("output", Property(this, &Table1D::getOutput));
  }
  virtual ~Table1D(void) {}
  
  virtual bool init(void)
  {
    OpenFDMAssert(getInputPort(0)->isConnected());
  
    return getInputPort(0)->isConnected();
  }

  virtual void output(const TaskInfo&)
  {
    OpenFDMAssert(getInputPort(0)->isConnected());
    TableData<1>::InterpVector interpVec;
    RealPortHandle rh = getInputPort(0)->toRealPortHandle();
    interpVec(1) = mTableLookup.lookup(rh.getRealValue());
    mOutput = mTableData.interpolate(interpVec);
  }

  const real_type& getOutput(void) const
  { return mOutput; }

  void setTableData(const TableData<1>& table)
  { mTableData = table; }
  const TableData<1>& getTableData(void) const 
  { return mTableData; }
  TableData<1>& getTableData(void)
  { return mTableData; }

  void setTableLookup(const TableLookup& tl)
  { mTableLookup = tl; }

private:
  real_type mOutput;
  TableData<1> mTableData;
  TableLookup mTableLookup;
};

class Table2D : public Model {
public:
  Table2D(const std::string& name) :
    Model(name)
  {
    setDirectFeedThrough(true);
    
    setNumInputPorts(2);
    setInputPortName(0, "input 0");
    setInputPortName(1, "input 1");
    
    setNumOutputPorts(1);
    setOutputPort(0, "output", this, &Table2D::getOutput);

    addProperty("output", Property(this, &Table2D::getOutput));
  }
  virtual ~Table2D(void) {}
  
  virtual bool init(void)
  {
    OpenFDMAssert(getInputPort(0)->isConnected());
    OpenFDMAssert(getInputPort(1)->isConnected());
    return getInputPort(0)->isConnected() && getInputPort(1)->isConnected();
  }

  virtual void output(const TaskInfo&)
  {
    OpenFDMAssert(getInputPort(0)->isConnected());
    TableData<2>::InterpVector interpVec;
    RealPortHandle rh = getInputPort(0)->toRealPortHandle();
    interpVec(1) = mTableLookup[0].lookup(rh.getRealValue());
    rh = getInputPort(1)->toRealPortHandle();
    interpVec(2) = mTableLookup[1].lookup(rh.getRealValue());
    mOutput = mTableData.interpolate(interpVec);
  }

  const real_type& getOutput(void) const
  { return mOutput; }

  void setTableData(const TableData<2>& table)
  { mTableData = table; }
  const TableData<2>& getTableData(void) const 
  { return mTableData; }
  TableData<2>& getTableData(void)
  { return mTableData; }

  void setTableLookup(unsigned idx, const TableLookup& tl)
  { mTableLookup[idx] = tl; }

private:
  real_type mOutput;
  TableData<2> mTableData;
  TableLookup mTableLookup[2];
};

class Table3D : public Model {
public:
  Table3D(const std::string& name) :
    Model(name)
  {
    setDirectFeedThrough(true);
    
    setNumInputPorts(3);
    setInputPortName(0, "input 0");
    setInputPortName(1, "input 1");
    setInputPortName(2, "input 2");
    
    setNumOutputPorts(1);
    setOutputPort(0, "output", this, &Table3D::getOutput);

    addProperty("output", Property(this, &Table3D::getOutput));
  }
  virtual ~Table3D(void) {}
  
  virtual bool init(void)
  {
    OpenFDMAssert(getInputPort(0)->isConnected());
    OpenFDMAssert(getInputPort(1)->isConnected());
    OpenFDMAssert(getInputPort(3)->isConnected());
    return getInputPort(0)->isConnected() && getInputPort(1)->isConnected()
      && getInputPort(2)->isConnected();
  }

  virtual void output(const TaskInfo&)
  {
    OpenFDMAssert(getInputPort(0)->isConnected());
    TableData<3>::InterpVector interpVec;
    RealPortHandle rh = getInputPort(0)->toRealPortHandle();
    interpVec(1) = mTableLookup[0].lookup(rh.getRealValue());
    rh = getInputPort(1)->toRealPortHandle();
    interpVec(2) = mTableLookup[1].lookup(rh.getRealValue());
    rh = getInputPort(2)->toRealPortHandle();
    interpVec(3) = mTableLookup[2].lookup(rh.getRealValue());
    mOutput = mTableData.interpolate(interpVec);
  }

  const real_type& getOutput(void) const
  { return mOutput; }

  void setTableData(const TableData<3>& table)
  { mTableData = table; }
  const TableData<3>& getTableData(void) const 
  { return mTableData; }
  TableData<3>& getTableData(void)
  { return mTableData; }

  void setTableLookup(unsigned idx, const TableLookup& tl)
  { mTableLookup[idx] = tl; }

private:
  real_type mOutput;
  TableData<3> mTableData;
  TableLookup mTableLookup[3];
};

} // namespace OpenFDM

#endif
