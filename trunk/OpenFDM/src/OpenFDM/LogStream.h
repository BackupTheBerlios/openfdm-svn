/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LogStream_H
#define OpenFDM_LogStream_H

#include <iosfwd>
#include <ostream>

namespace OpenFDM {

namespace Log {

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

template<Category category, Priority priority>
class LogFactory;

template<typename charT, typename traits = std::char_traits<charT> >
class LogProxy;

template<typename charT, typename traits = std::char_traits<charT> >
class LogSink;

template<class charT, class traits>
class LogProxy {
public:
  typedef std::ios_base                    ios_base_type;
  typedef std::basic_ios<charT,traits>     basic_ios_type;
  typedef std::basic_ostream<charT,traits> basic_ostream_type;

  template<typename T>
  LogProxy operator<<(const T& value)
  { if (mStream) (*mStream) << value; return *this; }
  LogProxy operator<<(ios_base_type& (*mptr)(ios_base_type&))
  { if (mStream) (*mStream) << mptr; return *this; }
  LogProxy operator<<(basic_ios_type& (*mptr)(basic_ios_type&))
  { if (mStream) (*mStream) << mptr; return *this; }
  LogProxy operator<<(basic_ostream_type& (*mptr)(basic_ostream_type&))
  { if (mStream) (*mStream) << mptr; return *this; }

private:
  template<Category category, Priority priority>
  friend class LogFactory;

  LogProxy(const LogProxy& lp) : mStream(lp.mStream) {}
  LogProxy(basic_ostream_type* s) : mStream(s) {}
  LogProxy& operator=(const LogProxy&);

  // A pointer to a stream we want to do output to.
  basic_ostream_type* mStream;
};

template<class charT, class traits>
class LogSink {
public:
  typedef std::ios_base                    ios_base_type;
  typedef std::basic_ios<charT,traits>     basic_ios_type;
  typedef std::basic_ostream<charT,traits> basic_ostream_type;

  template<typename T>
  LogSink operator<<(const T&)
  { return *this; }
  LogSink operator<<(ios_base_type& (*mptr)(ios_base_type&))
  { return *this; }
  LogSink operator<<(basic_ios_type& (*mptr)(basic_ios_type&))
  { return *this; }
  LogSink operator<<(basic_ostream_type& (*mptr)(basic_ostream_type&))
  { return *this; }

private:
  template<Category category, Priority priority>
  friend class LogFactory;

  LogSink(const LogSink&) {}
  LogSink(void) {}
  LogSink& operator=(const LogSink&);
};

class Logger {
public:
  static void setCategoryEnable(Category category, bool enable = true);
  static void setCategoryDisable(Category category);
  static void setPriority(Priority priority);

  bool enabled(Category category, Priority priority) const
  {
    return ((category & mCategory) && (priority <= mPriority))
      || (priority == Error);
  }

  std::basic_ostream<char>* getStream(void)
  { return mStream; }

protected:
  static Logger* Instance(void);

private:
  Logger(std::basic_ostream<char>* stream);

  std::basic_ostream<char>* mStream;
  unsigned mCategory;
  unsigned mPriority;
};

#if defined(NDEBUG)

template<Category category, Priority priority>
class LogFactory : public Logger {
public:
  typedef LogSink<char> proxy_type;
  static proxy_type
  CreateInstance(void)
  { return proxy_type(); }
};

template<Category category>
class LogFactory<category,Error> : public Logger {
public:
  typedef LogProxy<char> proxy_type;
  static proxy_type
  CreateInstance(void)
  {
    Logger* logger = Instance();
    if (logger->enabled(category, Error))
      return proxy_type(logger->getStream());
    else
      return proxy_type(0);
  }
};

template<Category category>
class LogFactory<category,Warning> : public Logger {
public:
  typedef LogProxy<char> proxy_type;
  static proxy_type
  CreateInstance(void)
  {
    Logger* logger = Instance();
    if (logger->enabled(category, Warning))
      return proxy_type(logger->getStream());
    else
      return proxy_type(0);
  }
};

#else /* defined(NDEBUG) */

template<Category category, Priority priority>
class LogFactory : public Logger {
public:
  static LogProxy<char>
  CreateInstance(void)
  {
    Logger* logger = Instance();
    if (logger->enabled(category, priority))
      return LogProxy<char>(logger->getStream());
    else
      return LogProxy<char>(0);
  }
};

#endif /* else defined(NDEBUG) */

} // namespace Log

#define Log(category, priority) \
  Log::LogFactory<Log::category,Log::priority>::CreateInstance()

using Log::Logger;

using std::endl;

} // namespace OpenFDM

#endif
