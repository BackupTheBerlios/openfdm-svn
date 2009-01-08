/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LogStream_H
#define OpenFDM_LogStream_H

#include <iosfwd>
// FIXME: because of using std::endl;
#include <ostream>

namespace OpenFDM {

class Logger {
public:
  enum Category {
    ArtBody          = 1,
    MultiBody        = ArtBody << 1,
    TimeStep         = MultiBody << 1,
    BoundCheck       = TimeStep << 1,
    Environment      = BoundCheck << 1,
    Frame            = Environment << 1,
    Initialization   = Frame << 1,
    NewtonMethod     = Initialization << 1,
    Misc             = NewtonMethod << 1,
    Model            = Misc << 1,
    Schedule         = Model << 1,
    Assert           = ~0
  };
  
  enum Priority {
    /// Non recoverable error, either due to an implementation problem or
    /// due to a user problem probably ignoring previous error return values.
    /// Simulation results may not be valid.
    Error            = 0,
    /// Error Conditions that should be avoided, but could be
    /// worked around in some way.
    /// Simulation results may not be valid.
    Warning          = Error + 1,
    /// Error Conditions that are marked with an error return.
    /// The exact reason is probably explained in this kind of messages.
    /// These kind of errors do not lead to immediate problems in the
    /// simulation code. Anyway, when not handled correctly they might lead to
    /// Error or Warning conditions.
    Info             = Warning + 1,
    /// Blubber for debugging ...
    Debug            = Info + 1,
    Debug1           = Debug + 1,
    Debug2           = Debug1 + 1,
    Debug3           = Debug2 + 1
  };
  
  static void setCategoryEnable(Category category, bool enable = true);
  static void setCategoryDisable(Category category);
  static void setPriority(Priority priority);

  static inline bool
  getStaticEnabled(Category category, Priority priority)
  {
#if defined(NDEBUG) || defined(_NDEBUG)
    // In the NDEBUG case, give the compilers optimizer a chance to
    // completely remove the code.
    if (Debug <= priority)
      return false;
#endif
    return Instance()->getEnabled(category, priority);
  }
  
  static std::ostream& getStream(Priority priority);

protected:
  static Logger* Instance(void);
  bool getEnabled(Category category, Priority priority);

private:
  Logger(std::ostream* stream = 0);

  std::ostream* mStream;
  unsigned mCategory;
  int mPriority;
};

#define Log(category, priority) \
if (Logger::getStaticEnabled(Logger::category, Logger::priority)) \
  Logger::getStream(Logger::priority)

using std::endl;

} // namespace OpenFDM

#endif
