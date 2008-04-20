/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CSVWriter_H
#define OpenFDM_CSVWriter_H

#include <fstream>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/ModelVisitor.h>
#include <OpenFDM/System.h>

namespace OpenFDM {

class CSVWriter : public ModelVisitor {
public:
  CSVWriter(const std::string& filename) :
    _csvFile(filename.c_str())
  { }
  virtual void apply(Model& model)
  {
    unsigned numOutputs = model.getNumOutputPorts();
    for (unsigned i = 0; i < numOutputs; ++i) {
      NumericPortProvider* numericPort = model.getOutputPort(i);
      if (!numericPort)
        continue;
      PortInterface* portInterface = numericPort->getPortInterface();
      if (!portInterface)
        continue;
      MatrixPortInterface* matrixPortInterface;
      matrixPortInterface = portInterface->toMatrixPortInterface();
      if (!matrixPortInterface)
        continue;

      const Matrix& m = matrixPortInterface->getMatrixValue();
      for (unsigned i = 0; i < rows(m); ++i)
        for (unsigned j = 0; j < cols(m); ++j)
          _csvFile << ", " << m(i, j);
    }
  }
  virtual void apply(ModelGroup& modelGroup)
  { traverse(modelGroup); }
  virtual void apply(System& system)
  {
    _csvFile << system.getTime();
    ModelVisitor::apply(system);
    _csvFile << std::endl;
  }
private:
  std::ofstream _csvFile;
};

} // namespace OpenFDM

#endif
