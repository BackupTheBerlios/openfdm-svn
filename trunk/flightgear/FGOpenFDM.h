/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef FGOpenFDM_H
#define FGOpenFDM_H

#include <FDM/flight.hxx>

namespace OpenFDM {

struct FGOpenFDMData;
class Object;
class ModelGroup;

///
/// Interface to an OpenFDM flightmodel.
///
class FGOpenFDM
  : public FGInterface {
public:
  FGOpenFDM(SGPropertyNode* fdmRootNode = 0);
  virtual ~FGOpenFDM(void);

  /// Initializes the fdm
  virtual void init();

  /// Connects the fdm with the flightgear property tree
  virtual void bind();

  /// Disconnects the fdm from the flightgear property tree
  virtual void unbind();
  
  /// Advances the FDM by dt seconds
  virtual void update(double dt);

private:
  static void untieNamed(SGPropertyNode* base, const char* name);
  static void untieRecursive(SGPropertyNode* base);

  FGOpenFDMData* mData;
  SGPropertyNode_ptr mAircraftRootNode;
};

} // namespace OpenFDM

#endif
