/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich
 *
 */

#ifndef OpenFDM_Table_H
#define OpenFDM_Table_H

#include <map>
#include "Matrix.h"
#include "Vector.h"

#include "Model.h"

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

class TablePreLookup : public Model {
public:
  TablePreLookup(const std::string& name) :
    Model(name)
  {
    setDirectFeedThrough(true);

    setNumInputPorts(1);
    setInputPortName(0, "input");
    
    setNumOutputPorts(1);
    setOutputPort(0, "output", this, &TablePreLookup::getOutput);

    addProperty("output", Property(this, &TablePreLookup::getOutput));
  }
  virtual ~TablePreLookup(void)
  { }

  virtual bool init(void)
  {
    mInputPortHandle = getInputPort(0)->toRealPortHandle();
    if (!mInputPortHandle.isConnected()) {
      Log(Model,Error) << "Input port to TablePreLookup Model \""
                       << getName() << "\" is not connected" << endl;
      return false;
    }
    return true;
  }

  virtual void output(const TaskInfo&)
  {
    OpenFDMAssert(mInputPortHandle.isConnected());
    mOutput = mTableLookup.lookup(mInputPortHandle.getRealValue());
    Log(Model,Debug3) << "Output of TablePreLookup \"" << getName() << "\" "
                      << mOutput << endl;
  }

  const real_type& getOutput(void) const
  { return mOutput; }

  void setTableLookup(const TableLookup& tl)
  { mTableLookup = tl; }
  const TableLookup& getTableLookup(void) const
  { return mTableLookup; }

private:
  real_type mOutput;
  TableLookup mTableLookup;
  RealPortHandle mInputPortHandle;
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
    mInputPortHandle = getInputPort(0)->toRealPortHandle();
    if (!mInputPortHandle.isConnected()) {
      Log(Model,Error) << "Input port to Table1D Model \""
                       << getName() << "\" is not connected" << endl;
      return false;
    }
    return true;
  }

  virtual void output(const TaskInfo&)
  {
    OpenFDMAssert(mInputPortHandle.isConnected());
    TableData<1>::InterpVector interpVec;
    interpVec(1) = mInputPortHandle.getRealValue();
    mOutput = mTableData.interpolate(interpVec);
    Log(Model,Debug3) << "Output of Table1D \"" << getName() << "\" "
                      << mOutput << endl;
  }

  const real_type& getOutput(void) const
  { return mOutput; }

  void setTableData(const TableData<1>& table)
  { mTableData = table; }
  const TableData<1>& getTableData(void) const 
  { return mTableData; }
  TableData<1>& getTableData(void)
  { return mTableData; }

private:
  real_type mOutput;
  TableData<1> mTableData;
  RealPortHandle mInputPortHandle;
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
    for (unsigned idx = 0; idx < 2; ++idx) {
      mInputPortHandle[idx] = getInputPort(idx)->toRealPortHandle();
      if (!mInputPortHandle[idx].isConnected()) {
        Log(Model,Error) << "Input port to Table2D Model \""
                         << getName() << "\" is not connected" << endl;
        return false;
      }
    }
    return true;
  }

  virtual void output(const TaskInfo&)
  {
    OpenFDMAssert(mInputPortHandle[0].isConnected());
    OpenFDMAssert(mInputPortHandle[1].isConnected());
    TableData<2>::InterpVector interpVec;
    interpVec(1) = mInputPortHandle[0].getRealValue();
    interpVec(2) = mInputPortHandle[1].getRealValue();
    mOutput = mTableData.interpolate(interpVec);
    Log(Model, Debug3) << "Output of Table2D \"" << getName() << "\" "
                       << mOutput << endl;
  }

  const real_type& getOutput(void) const
  { return mOutput; }

  void setTableData(const TableData<2>& table)
  { mTableData = table; }
  const TableData<2>& getTableData(void) const 
  { return mTableData; }
  TableData<2>& getTableData(void)
  { return mTableData; }

private:
  real_type mOutput;
  TableData<2> mTableData;
  RealPortHandle mInputPortHandle[2];
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
    for (unsigned idx = 0; idx < 3; ++idx) {
      mInputPortHandle[idx] = getInputPort(idx)->toRealPortHandle();
      if (!mInputPortHandle[idx].isConnected()) {
        Log(Model,Error) << "Input port to Table3D Model \""
                         << getName() << "\" is not connected" << endl;
        return false;
      }
    }
    return true;
  }

  virtual void output(const TaskInfo&)
  {
    OpenFDMAssert(mInputPortHandle[0].isConnected());
    OpenFDMAssert(mInputPortHandle[1].isConnected());
    OpenFDMAssert(mInputPortHandle[2].isConnected());
    TableData<3>::InterpVector interpVec;
    interpVec(1) = mInputPortHandle[0].getRealValue();
    interpVec(2) = mInputPortHandle[1].getRealValue();
    interpVec(3) = mInputPortHandle[2].getRealValue();
    mOutput = mTableData.interpolate(interpVec);
    Log(Model, Debug3) << "Output of Table3D \"" << getName() << "\" "
                       << mOutput << endl;
  }

  const real_type& getOutput(void) const
  { return mOutput; }

  void setTableData(const TableData<3>& table)
  { mTableData = table; }
  const TableData<3>& getTableData(void) const 
  { return mTableData; }
  TableData<3>& getTableData(void)
  { return mTableData; }

private:
  real_type mOutput;
  TableData<3> mTableData;
  RealPortHandle mInputPortHandle[3];
};

} // namespace OpenFDM

#endif
