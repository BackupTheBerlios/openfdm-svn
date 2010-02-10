/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ReaderWriter_H
#define OpenFDM_ReaderWriter_H

#include <list>
#include <string>

#include "Object.h"

namespace OpenFDM {

class System;

/** Abstract base class for a reader/writer of OpenFDM vehicles.
 * (At present only reading ...) FIXME
 */
class ReaderWriter {
public:
  /** Errors are just string values.
   */
  typedef std::list<std::string> StringList;

  /** Constructor.
   */
  ReaderWriter(void);
  /** Destructor.
   */
  virtual ~ReaderWriter(void);

  /** Returns true if there was an error up to now.
   */
  bool getErrorState(void) const
  { return 0 < mErrors.size(); }

  /** Return a list of all error messages.
   */
  const StringList& getErrors(void) const
  { return mErrors; }

  /** Returns true if there was an error up to now.
   */
  void resetErrorState(void);

  /** Returns a pointer to the vehicle.
   */
  System* getSystem(void)
  { return mSystem; }
  
protected:
  /** Slot where a ReaderWriter implementation should reset it's state.
   */
  virtual void reset(void);

  /** Emit an error.
   * New errors are pushed in front of the error list.
   * This way the list of error contains a series of messages starting from
   * a top level information down to the detail where it happend.
   * This functiion always returns false. This is just for conveinience to be
   * able to write 'return error("whatever");' in the code.
   */
  bool error(const std::string& message);

  /** The System.
   */
  SharedPtr<System> mSystem;

private:
  /** A list of errors during import.
   */
  StringList mErrors;
};

} // namespace OpenFDM

#endif
