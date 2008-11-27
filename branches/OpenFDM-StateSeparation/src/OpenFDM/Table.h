/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich
 *
 */

#ifndef OpenFDM_Table_H
#define OpenFDM_Table_H

#include "TableData.h"
#include "Model.h"
#include "SimpleDirectModel.h"

namespace OpenFDM {

class BreakPointLookup : public SimpleDirectModel {
  OPENFDM_OBJECT(BreakPointLookup, SimpleDirectModel);
public:
  BreakPointLookup(const std::string& name);
  virtual ~BreakPointLookup(void);

  virtual void output(Context& context) const;

  void setBreakPointVector(const BreakPointVector& breakPointVector)
  { mBreakPointVector = breakPointVector; }
  const BreakPointVector& getBreakPointVector(void) const
  { return mBreakPointVector; }

private:
  BreakPointVector mBreakPointVector;
};

class Table1D : public SimpleDirectModel {
  OPENFDM_OBJECT(Table1D, SimpleDirectModel);
public:
  Table1D(const std::string& name);
  virtual ~Table1D(void);
  
  virtual void output(Context& context) const;

  void setTableData(const TableData<1>& table)
  { mTableData = table; }
  const TableData<1>& getTableData(void) const 
  { return mTableData; }
  TableData<1>& getTableData(void)
  { return mTableData; }

private:
  TableData<1> mTableData;
};

class Table2D : public SimpleDirectModel {
  OPENFDM_OBJECT(Table2D, SimpleDirectModel);
public:
  Table2D(const std::string& name);
  virtual ~Table2D(void);
  
  virtual void output(Context& context) const;

  void setTableData(const TableData<2>& table)
  { mTableData = table; }
  const TableData<2>& getTableData(void) const 
  { return mTableData; }
  TableData<2>& getTableData(void)
  { return mTableData; }

private:
  TableData<2> mTableData;
};

class Table3D : public SimpleDirectModel {
  OPENFDM_OBJECT(Table3D, SimpleDirectModel);
public:
  Table3D(const std::string& name);
  virtual ~Table3D(void);
  
  virtual void output(Context& context) const;

  void setTableData(const TableData<3>& table)
  { mTableData = table; }
  const TableData<3>& getTableData(void) const 
  { return mTableData; }
  TableData<3>& getTableData(void)
  { return mTableData; }

private:
  TableData<3> mTableData;
};

} // namespace OpenFDM

#endif
