/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimFCSComponent_H
#define OpenFDM_JSBSimFCSComponent_H

#include <string>
#include <OpenFDM/Referenced.h>
#include <OpenFDM/SharedPtr.h>
#include <OpenFDM/Model.h>
#include <OpenFDM/GroupOutput.h>

namespace OpenFDM {

class Group;
class Gain;
class Port;

/// Just a small container mapping the JSBSim FCS component strangeness to
/// the available OpenFDM discrete systems.
/// As JSBSim usually munges several independent things into one thing,
/// this is a Group for the first cut.
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
  const Port* getOutputPort(void);
  const Port* getInternalOutputPort(void);
  /// Return /the/ normalized output port
  const Port* getOutputNormPort(void);
  const Port* getInternalOutputNormPort(void);

  /// Return /the/ group interface nodes
  GroupOutput* getOutputModel(void);
  GroupOutput* getOutputNormModel(void);

  Group* getGroup(void)
  { return mGroup; }
private:
  SharedPtr<Group> mGroup;
  SharedPtr<GroupOutput> mOutputModel;
  SharedPtr<GroupOutput> mOutputNormModel;
};

} //namespace OpenFDM

#endif
