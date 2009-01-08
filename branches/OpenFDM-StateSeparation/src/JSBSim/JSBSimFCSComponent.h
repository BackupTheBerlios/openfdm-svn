/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimFCSComponent_H
#define OpenFDM_JSBSimFCSComponent_H

#include <string>
#include <OpenFDM/Referenced.h>
#include <OpenFDM/SharedPtr.h>
#include <OpenFDM/Model.h>

namespace OpenFDM {

class ModelGroup;
class Gain;
class Port;

/// Just a small container mapping the JSBSim FCS component strangeness to
/// the available OpenFDM discrete systems.
/// As JSBSim usually munges several independent things into one thing,
/// this is a ModelGroup for the first cut.
class JSBSimFCSComponent :
    public Referenced {
public:
  JSBSimFCSComponent(const std::string& name);
  virtual ~JSBSimFCSComponent(void);

//   /// Add an inverter into the input chanel numbered inNumber
//   void invertInput(unsigned inNumber);
//   /// Add an inverter into the output chanel
//   void invertOutput(void);

  /// Return /the/ output port (we have only one)
  NumericPortProvider* getOutputPort(void);
  NumericPortAcceptor* getInternalOutputPort(void);
  /// Return /the/ normalized output port
  NumericPortProvider* getOutputNormPort(void);
  NumericPortAcceptor* getInternalOutputNormPort(void);

  ModelGroup* getModelGroup(void)
  { return mModelGroup; }
private:
  SharedPtr<ModelGroup> mModelGroup;
  SharedPtr<NumericPortAcceptor> mInternalOutputPort;
  SharedPtr<NumericPortAcceptor> mInternalOutputNormPort;
};

} //namespace OpenFDM

#endif
