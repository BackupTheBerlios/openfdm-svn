/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Tablereader_H
#define OpenFDM_Tablereader_H

#include <iosfwd>

#include <OpenFDM/Table.h>

namespace OpenFDM {

bool
parseTable1D(std::istream& s, TableData<1>& data, BreakPointVector& lookup);

bool
parseTable2D(std::istream& s, TableData<2>& data, BreakPointVector lookup[2]);

bool
parseTable3D(std::istream& s, TableData<3>& data, BreakPointVector lookup[3]);

} // namespace OpenFDM

#endif
