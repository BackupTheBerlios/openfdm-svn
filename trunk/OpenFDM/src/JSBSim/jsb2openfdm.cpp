#include <iostream>

#include <OpenFDM/ReaderWriter.h>
#include "LegacyJSBSimReader.h"

using OpenFDM::ReaderWriter;
using OpenFDM::LegacyJSBSimReader;

int
main(int argc, char *argv[])
{
  if (argc < 3)
    return 1;

  // Try to read JSBSim legacy files.
  LegacyJSBSimReader reader;

  reader.addAircraftPath(argv[1]);
  reader.addEnginePath(std::string(argv[1]) + "Engines/");

  reader.loadAircraft(argv[2]);
  if (reader.getErrorState()) {
    std::cerr << "FGOpenFDM::init() cannot read aircraft!" << std::endl;
    const ReaderWriter::StringList errors = reader.getErrors();
    ReaderWriter::StringList::const_iterator it;
    for (it = errors.begin(); it != errors.end(); ++it)
      std::cerr << *it << std::endl;

    return 1;
  }

  // Ok, now the Vehicle here contains the imported data
  // When the reflection stuff is ready, we can dump that data to a
  // native format ...
  // reader.getVehicle();

  return 0;
}
