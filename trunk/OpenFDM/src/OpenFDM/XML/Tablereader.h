/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Tablereader_H
#define OpenFDM_Tablereader_H

#include <iosfwd>

#include <OpenFDM/Table.h>

namespace OpenFDM {

bool
parseTable1D(std::istream& s, TableData<1>& data, TableLookup& lookup);

bool
parseTable2D(std::istream& s, TableData<2>& data, TableLookup lookup[2]);

bool
parseTable3D(std::istream& s, TableData<3>& data, TableLookup lookup[3]);

} // namespace OpenFDM

#endif
