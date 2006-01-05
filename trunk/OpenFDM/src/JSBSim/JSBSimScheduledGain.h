/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimScheduledGain_H
#define OpenFDM_JSBSimScheduledGain_H

#include "JSBSimFCSComponent.h"

namespace OpenFDM {

class Table1D;
template<unsigned>
class TableData;
class TableLookup;

/// Just a small container mapping the JSBSim ScheduledGain parameters to
/// the OpenFDM models.
class JSBSimScheduledGain :
    public JSBSimFCSComponent {
public:
  JSBSimScheduledGain(const std::string& name);
  virtual ~JSBSimScheduledGain(void);

  void setTableData(const TableData<1>& tableData, const TableLookup& lookup);

private:
  SharedPtr<TablePreLookup> mTablePreLookup;
  SharedPtr<Table1D> mTable;
};

} //namespace OpenFDM

#endif
