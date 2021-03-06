/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Tablereader.h"

#include <iostream>

namespace OpenFDM {

bool
parseTable1D(std::istream& s, TableData<1>& data, BreakPointVector& lookup)
{
  /// FIXME error handling
  for (unsigned i = 0; s && i < data.size(0); ++i) {
    real_type in = 0;
    real_type out = 0;
    s >> in >> out;
    lookup.insert(in);
    TableData<1>::Index iv;
    iv(0) = i;
    data(iv) = out;
  }
  return lookup.size() == data.size(0) && s;
}

bool
parseTable2D(std::istream& s, TableData<2>& data, BreakPointVector lookup[2])
{
  /// FIXME error handling
  for (unsigned j = 0; s && j < data.size(1); ++j) {
    real_type in = 0;
    s >> in;
    lookup[1].insert(in);
  }

  if (lookup[1].size() != data.size(1))
    return false;
  
  for (unsigned i = 0; s && i < data.size(0); ++i) {
    real_type in = 0;
    s >> in;

    lookup[0].insert(in);

    for (unsigned j = 0; s && j < data.size(1); ++j) {
      real_type out = 0;
      s >> out;

      TableData<2>::Index iv;
      iv(0) = i;
      iv(1) = j;
      data(iv) = out;
    }
  }
  return lookup[0].size() == data.size(0) && s;
}

bool
parseTable3D(std::istream& s, TableData<3>& data, BreakPointVector lookup[3])
{
  for (unsigned k = 0; s && k < data.size(2); ++k) {
    real_type in = 0;
    s >> in;
    lookup[2].insert(in);

    for (unsigned j = 0; s && j < data.size(1); ++j) {
      real_type in = 0;
      s >> in;
      lookup[1].insert(in);
    }
    if (lookup[1].size() != data.size(1))
      return false;
    
    for (unsigned i = 0; s && i < data.size(0); ++i) {
      real_type in = 0;
      s >> in;
      
      lookup[0].insert(in);
      
      for (unsigned j = 0; s && j < data.size(1); ++j) {
        real_type out = 0;
        s >> out;
        
        TableData<3>::Index iv;
        iv(0) = i;
        iv(1) = j;
        iv(2) = k;
        data(iv) = out;
      }
    }
  }
  return lookup[0].size() == data.size(0) &&
    lookup[2].size() == data.size(2) && s;
}

} // namespace OpenFDM
