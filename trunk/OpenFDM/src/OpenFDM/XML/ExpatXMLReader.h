/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "XMLReader.h"

namespace OpenFDM {
namespace XML {

class ExpatXMLReader : public XMLReader {
public:
  ExpatXMLReader(void);
  virtual ~ExpatXMLReader(void);
  // returns bool instead of void to signal errors
  virtual void parse(std::istream& stream);
};

} // namespace XML
} // namespace OpenFDM
