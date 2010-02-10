/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XML_EasyXMLReader_H
#define OpenFDM_XML_EasyXMLReader_H

#include "XMLReader.h"

namespace OpenFDM {
namespace XML {

class EasyXMLReader : public XMLReader {
public:
  EasyXMLReader(void);
  virtual ~EasyXMLReader(void);
  // returns bool instead of void to signal errors
  virtual void parse(std::istream& stream);
};

} // namespace XML
} // namespace OpenFDM

#endif
