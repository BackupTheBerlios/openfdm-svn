/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimScheduledGain_H
#define OpenFDM_JSBSimScheduledGain_H

#include "JSBSimFCSComponent.h"
#include <OpenFDM/SharedPtr.h>
#include <OpenFDM/Table.h>

namespace OpenFDM {

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
