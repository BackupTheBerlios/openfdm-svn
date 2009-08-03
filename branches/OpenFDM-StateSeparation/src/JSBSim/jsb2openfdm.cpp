#include <iostream>

#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/System.h>
#include <OpenFDM/XMLDumpModelVisitor.h>
#include "JSBSimReader.h"

using OpenFDM::ReaderWriter;
using OpenFDM::JSBSimReader;
using OpenFDM::SharedPtr;
using OpenFDM::System;

int
main(int argc, char *argv[])
{
  if (argc < 3) {
    std::cerr << "No aircraft given!" << std::endl;
    return 1;
  }

  std::string aircraftDir = argv[1];
  std::string engineDir = aircraftDir + "/Engines";
  std::string aircraftFile = argv[2];

  JSBSimReader reader;
  reader.addAircraftPath(aircraftDir);
  reader.addEnginePath(engineDir);
  reader.loadAircraft(aircraftFile);
  if (reader.getErrorState()) {
    std::cerr << "Cannot read aircraft!" << std::endl;
    ReaderWriter::StringList errors = reader.getErrors();
    ReaderWriter::StringList::const_iterator it;
    for (it = errors.begin(); it != errors.end(); ++it)
      std::cerr << (*it) << std::endl;
    
    return EXIT_FAILURE;
  }
  SharedPtr<System> system = reader.getSystem();
  if (!system) {
    std::cerr << "No imported aircraft system!" << std::endl;
    return EXIT_FAILURE;
  }

  if (!system->init()) {
    std::cerr << "Could not initialize aircraft system!" << std::endl;
    return EXIT_FAILURE;
  }
  
  // Ok, now the Vehicle here contains the imported data
  // When the reflection stuff is ready, we can dump that data to a
  // native format ...
  OpenFDM::XMLDumpModelVisitor debugDumpVisitor(std::cout);
  debugDumpVisitor.apply(*system);

  return 0;
}
