/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <iostream>

#include "Tablereader.h"

namespace OpenFDM {

bool
parseTable1D(std::istream& s, TableData<1>& data, TableLookup& lookup)
{
  /// FIXME error handling
  for (unsigned i = 1; s && i <= data.size(1); ++i) {
    real_type in = 0;
    real_type out = 0;
    s >> in >> out;
    lookup.setAtIndex(i, in);
    TableData<1>::Index iv;
    iv(1) = i;
    data(iv) = out;
  }
  return lookup.isValid() && lookup.size() == data.size(1) && s;
}

bool
parseTable2D(std::istream& s, TableData<2>& data, TableLookup lookup[2])
{
  /// FIXME error handling
  for (unsigned j = 1; s && j <= data.size(2); ++j) {
    real_type in = 0;
    s >> in;
    lookup[1].setAtIndex(j, in);
  }

  if (!lookup[1].isValid() || lookup[1].size() != data.size(2))
    return false;
  
  for (unsigned i = 1; s && i <= data.size(1); ++i) {
    real_type in = 0;
    s >> in;

    lookup[0].setAtIndex(i, in);

    for (unsigned j = 1; s && j <= data.size(2); ++j) {
      real_type out = 0;
      s >> out;

      TableData<2>::Index iv;
      iv(1) = i;
      iv(2) = j;
      data(iv) = out;
    }
  }
  return lookup[0].isValid() && lookup[0].size() == data.size(1) && s;
}

bool
parseTable3D(std::istream& s, TableData<3>& data, TableLookup lookup[3])
{
  for (unsigned k = 1; s && k <= data.size(3); ++k) {
    real_type in = 0;
    s >> in;
    lookup[2].setAtIndex(k, in);

    for (unsigned j = 1; s && j <= data.size(2); ++j) {
      real_type in = 0;
      s >> in;
      lookup[1].setAtIndex(j, in);
    }
    if (!lookup[1].isValid() || lookup[1].size() != data.size(2))
      return false;
    
    for (unsigned i = 1; s && i <= data.size(1); ++i) {
      real_type in = 0;
      s >> in;
      
      lookup[0].setAtIndex(i, in);
      
      for (unsigned j = 1; s && j <= data.size(2); ++j) {
        real_type out = 0;
        s >> out;
        
        TableData<3>::Index iv;
        iv(1) = i;
        iv(2) = j;
        iv(3) = k;
        data(iv) = out;
      }
    }
  }
  return lookup[0].isValid() && lookup[0].size() == data.size(1) &&
    lookup[2].isValid() && lookup[2].size() == data.size(3) && s;
}

} // namespace OpenFDM
