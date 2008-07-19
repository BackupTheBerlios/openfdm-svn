/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich
 *
 */

#ifndef OpenFDM_Table_H
#define OpenFDM_Table_H

#include "TableData.h"
#include "Model.h"

namespace OpenFDM {

class TablePreLookup : public Model {
  OPENFDM_OBJECT(TablePreLookup, Model);
public:
  TablePreLookup(const std::string& name);
  virtual ~TablePreLookup(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getOutput(void) const;

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
  OPENFDM_OBJECT(Table1D, Model);
public:
  Table1D(const std::string& name);
  virtual ~Table1D(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getOutput(void) const;

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
  OPENFDM_OBJECT(Table2D, Model);
public:
  Table2D(const std::string& name);
  virtual ~Table2D(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getOutput(void) const;

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
  OPENFDM_OBJECT(Table3D, Model);
public:
  Table3D(const std::string& name);
  virtual ~Table3D(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getOutput(void) const;

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
