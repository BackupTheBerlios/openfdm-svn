/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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
    Error            = 0,
    Warning          = Error + 1,
    Info             = Warning + 1,
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
    if (Info <= priority)
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
