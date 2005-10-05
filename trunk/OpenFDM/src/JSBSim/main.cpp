#include <iostream>
#include <fstream>
#include <sstream>
#include <OpenFDM/XML/XMLReader.h>

using namespace std;
using namespace OpenFDM;

XMLElement*
scalarElement(const std::string& name,
              const std::string& value,
              const std::string& unit = "")
{
  XMLElement* element = new XMLElement(name);
  element->setData(value);
  if (unit != "")
    element->setAttribute("unit", unit);
  return element;
}

XMLElement*
vectorElement(const std::string& name,
              const std::string& value_x,
              const std::string& value_y,
              const std::string& value_z,
              const std::string& unit = "",
              const std::string& refFrame = "")
{
  XMLElement* element = new XMLElement(name);
  if (unit != "")
    element->setAttribute("unit", unit);
  if (refFrame != "")
    element->setAttribute("referenceFrame", refFrame);

  element->appendChild(scalarElement("x", value_x));
  element->appendChild(scalarElement("y", value_y));
  element->appendChild(scalarElement("z", value_z));

  return element;
}

class JSBImport {
public:
  JSBImport(void)
  {
    mJSBPrefix = "fdm/jsbsim/";
    mFCSPrefix = "fcs/";
  }

  void setAircraftPath(const std::string& aircraftPath);
  void setEnginePath(const std::string& enginePath);
  bool loadAircraft(const std::string& acConfig);
  
  XMLDocument::pointer getConverted(void);

private:
  std::string normalizeName(const std::string& name);
  std::string FCSName(const std::string& name);

  void fixupTEST(std::string& s);

  void parseMetrics(const std::string& data, XMLElement* top);
  void parseUndercarriage(const std::string& data, XMLElement* top);

  XMLElement::pointer parsePropulsion(const std::string& data);
  void convertPropulsion(XMLElement* newTopElem, XMLElement* propElem);
  void parseFCSComponent(const std::string& type, const std::string& name,
                         const std::string& data, XMLElement* fcs);
  void convertFCS(XMLElement* newFcElem, XMLElement* fcElem);
  void parseProductTable(XMLElement* prodElem, const std::string& type,
                         const std::string& data);
  void convertAEROSummands(XMLElement* newAeroSummands, XMLElement* aeroSummands);
  void convertAERO(XMLElement* newAeroElem, XMLElement* aeroElem);

  XMLDocument::pointer convert(XMLDocument::pointer jsbDoc);

  std::string mAircraftPath;
  std::string mEnginePath;

  std::string mPrevFCSOut;
  std::string mJSBPrefix;
  std::string mFCSPrefix;

  shared_ptr<XMLDocument> mConverted;
  shared_ptr<XMLElement> mRigidBody;
};

void
JSBImport::setAircraftPath(const std::string& aircraftPath)
{
  mAircraftPath = aircraftPath;
}

void
JSBImport::setEnginePath(const std::string& enginePath)
{
  mEnginePath = enginePath;
}

bool
JSBImport::loadAircraft(const std::string& acConfig)
{
  std::string infilename = mAircraftPath + acConfig;
  std::ifstream infile;
  infile.open(infilename.c_str());
  if (!infile.is_open())
    return false;

  std::stringstream sstr;
  infile >> sstr.rdbuf();
  infile.close();
  
  std::string s = sstr.str();
  fixupTEST(s);
  sstr.str(s);

  XMLDomParser parser;
  if (!parser.parseXML(sstr))
    return false;

  mConverted = convert(parser.getDocument());

  return mConverted;
}

XMLDocument::pointer
JSBImport::getConverted(void)
{
  return mConverted;
}

std::string
JSBImport::normalizeName(const std::string& name)
{
  std::string ret;
  std::string::const_iterator it = name.begin();
  while (it != name.end()) {
    if (isspace(*it) || *it == '_')
      ret += '-';
    else
      ret += tolower(*it);
    ++it;
  }
  return ret;
}

std::string
JSBImport::FCSName(const std::string& name)
{
  if (0 < name.size() && name[0] == '/')
    return name.substr(1);
  else
    return mJSBPrefix + name;
}

void
JSBImport::fixupTEST(std::string& s)
{
  std::string::size_type pos = 0;
  while ((pos = s.find("<TEST", pos)) != std::string::npos) {
    pos = s.find(">", pos);
    if (pos == std::string::npos)
      break;
    ++pos;
    std::string::size_type testend = s.find("</TEST>", pos);

    std::string::size_type reppos;
    while ((reppos = s.find(">", pos)) < testend)
      s.replace(reppos, 1, "&gt;"), testend += 3;
    while ((reppos = s.find(">=", pos)) < testend)
      s.replace(reppos, 2, "&gt;="), testend += 3;
    while ((reppos = s.find("<", pos)) < testend)
      s.replace(reppos, 1, "&lt;"), testend += 3;
    while ((reppos = s.find("<=", pos)) < testend)
      s.replace(reppos, 2, "&lt;="), testend += 3;

    pos = testend;
  }
}

void
JSBImport::parseMetrics(const std::string& data, XMLElement* top)
{
  std::stringstream data_stream(data);

  XMLElement::pointer element(new XMLElement("metrics"));

  XMLElement::pointer ielem(new XMLElement("inertia"));
  ielem->setAttribute("unit", "slug ft2");
  shared_ptr<XMLElement> massElement = new XMLElement("mass");
  massElement->appendChild(ielem);

  for (;;) {
    std::string name;
    data_stream >> name;
    if (!data_stream)
      break;
    
    if (name == "AC_WINGAREA") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("wingarea", val, "ft2"));
    } else if (name == "AC_WINGSPAN") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("wingspan", val, "ft"));
    } else if (name == "AC_WINGINCIDENCE") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("wingincidence", val, "deg"));
    } else if (name == "AC_CHORD") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("choord", val, "ft"));
    } else if (name == "AC_HTAILAREA") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("htailarea", val, "ft2"));
    } else if (name == "AC_HTAILARM") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("htailarm", val, "ft"));
    } else if (name == "AC_VTAILAREA") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("vtailarea", val, "ft2"));
    } else if (name == "AC_VTAILARM" || name == "AC_LV") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("vtailarm", val, "ft"));
    } else if (name == "AC_IXX") {
      std::string val;
      data_stream >> val;
      ielem->appendChild(scalarElement("ixx", val));
    } else if (name == "AC_IYY") {
      std::string val;
      data_stream >> val;
      ielem->appendChild(scalarElement("iyy", val));
    } else if (name == "AC_IZZ") {
      std::string val;
      data_stream >> val;
      ielem->appendChild(scalarElement("izz", val));
    } else if (name == "AC_IXY") {
      std::string val;
      data_stream >> val;
      ielem->appendChild(scalarElement("ixy", val));
    } else if (name == "AC_IXZ") {
      std::string val;
      data_stream >> val;
      ielem->appendChild(scalarElement("ixz", val));
    } else if (name == "AC_IYZ") {
      std::string val;
      data_stream >> val;
      ielem->appendChild(scalarElement("iyz", val));
    } else if (name == "AC_EMPTYWT") {
      std::string val;
      data_stream >> val;
      massElement->appendChild(scalarElement("mass", val, "lbs"));
    } else if (name == "AC_CGLOC") {
      std::string val[3];
      data_stream >> val[0] >> val[1] >> val[2];
      massElement->appendChild(vectorElement("position",
                                             val[0], val[1], val[2],
                                             "in", "structural"));

    } else if (name == "AC_EYEPTLOC") {
      std::string val[3];
      data_stream >> val[0] >> val[1] >> val[2];
      element->appendChild(vectorElement("eyepoint", val[0], val[1], val[2],
                                         "in", "structural"));
    } else if (name == "AC_AERORP") {
      std::string val[3];
      data_stream >> val[0] >> val[1] >> val[2];
      element->appendChild(vectorElement("aerorp", val[0], val[1], val[2],
                                         "in", "structural"));
    } else if (name == "AC_VRP") {
      std::string val[3];
      data_stream >> val[0] >> val[1] >> val[2];
      element->appendChild(vectorElement("vrp", val[0], val[1], val[2],
                                         "in", "structural"));
    } else if (name == "AC_POINTMASS") {
      std::string val[4];
      data_stream >> val[0] >> val[1] >> val[2] >> val[3];
      shared_ptr<XMLElement> pmelem = new XMLElement("mass");
      pmelem->appendChild(scalarElement("mass", val[0], "lbs"));
      pmelem->appendChild(vectorElement("position", val[1], val[2], val[3],
                                        "in", "structural"));
      mRigidBody->appendChild(pmelem);
    } else {
      std::cerr << "Unknown configuration \"" << name
                << "\" in METRICS section." << std::endl;
    }
  }

  mRigidBody->appendChild(massElement);
  top->appendChild(element);
}

void
JSBImport::parseUndercarriage(const std::string& data, XMLElement* top)
{
  std::stringstream data_stream(data);

  shared_ptr<XMLElement> element = new XMLElement("undercarriage");

  // Undercarriage parsing.
  for (;;) {
    std::string uctype;
    data_stream >> uctype;
    if (!data_stream)
      break;
    
    if (uctype == "AC_GEAR") {
      string name, type, brake, retract;
      string x, y, z, k, d, fs, fd, rr, sa;
      data_stream >> name >> x >> y >> z >> k >> d >> fs >> fd >> rr
                  >> type >> brake >> sa >> retract;

      shared_ptr<XMLElement> gelem = new XMLElement("gear");
      gelem->appendChild(scalarElement("name", name));
      gelem->appendChild(vectorElement("loc", x, y, z, "in", "structural"));
      gelem->appendChild(scalarElement("spring-constant", k, "lbs ft-1"));
      gelem->appendChild(scalarElement("spring-damping", d, "lbs s ft-1"));
      gelem->appendChild(scalarElement("static-friction", fs, "lbs s ft-1"));
      gelem->appendChild(scalarElement("dynamic-friction", fd, "lbs s ft-1"));
      gelem->appendChild(scalarElement("rolling-friction", rr, "lbs s ft-1"));
      gelem->appendChild(scalarElement("steerable", type));
      gelem->appendChild(scalarElement("brake-group", brake));
      gelem->appendChild(scalarElement("steering-angle", sa, "deg"));
      gelem->appendChild(scalarElement("retractable", retract));
      element->appendChild(gelem);

    } else if (uctype == "AC_CONTACT") {
      string name, type, brake, retract;
      string x, y, z, k, d, fs, fd, rr, sa;
      data_stream >> name >> x >> y >> z >> k >> d >> fs >> fd >> rr
                  >> type >> brake >> sa >> retract;

      shared_ptr<XMLElement> gelem = new XMLElement("contact");
      gelem->appendChild(scalarElement("name", name));
      gelem->appendChild(vectorElement("loc", x, y, z, "in", "structural"));
      gelem->appendChild(scalarElement("spring-constant", k, "lbs ft-1"));
      gelem->appendChild(scalarElement("spring-damping", d, "lbs s ft-1"));
      gelem->appendChild(scalarElement("static-friction", fs, "lbs s ft-1"));
      gelem->appendChild(scalarElement("dynamic-friction", fd, "lbs s ft-1"));
      element->appendChild(gelem);

    } else if (uctype == "AC_LAUNCHBAR") {
      string name, x, y, z, len, d;
      data_stream >> name >> x >> y >> z >> len >> d >> d >> d >> d
                  >> d >> d >> d >> d;

      shared_ptr<XMLElement> lbelem = new XMLElement("launchbar");
      lbelem->appendChild(scalarElement("name", name));
      lbelem->appendChild(vectorElement("loc", x, y, z, "in", "structural"));
      lbelem->appendChild(scalarElement("length", len, "in"));
      element->appendChild(lbelem);

    } else if (uctype == "AC_HOOK") {
      string name, x, y, z, len, upang, downang, d;
      data_stream >> name >> x >> y >> z >> len >> upang >> downang >> d >> d
                  >> d >> d >> d >> d;

      shared_ptr<XMLElement> helem = new XMLElement("hook");
      helem->appendChild(scalarElement("name", name));
      helem->appendChild(vectorElement("loc", x, y, z, "in", "structural"));
      helem->appendChild(scalarElement("length", len, "in"));
      helem->appendChild(scalarElement("up-angle", upang, "deg"));
      helem->appendChild(scalarElement("down-angle", downang, "deg"));
      element->appendChild(helem);

    } else {
      string d;
      data_stream >> d >> d >> d >> d >> d >> d >> d
                  >> d >> d >> d >> d >> d >> d;
      cout << uctype << " is not supported" << endl;
    }
  }

  top->appendChild(element);
}


shared_ptr<XMLElement>
JSBImport::parsePropulsion(const std::string& data)
{
  std::stringstream data_stream(data);

  shared_ptr<XMLElement> element = new XMLElement("propulsion");

  std::string loc[3];
  std::string pitch, yaw;
  for (;;) {
    std::string name;
    data_stream >> name;
    if (!data_stream)
      break;
    
    if (name == "XLOC") {
      data_stream >> loc[0];
    } else if (name == "YLOC") {
      data_stream >> loc[1];
    } else if (name == "ZLOC") {
      data_stream >> loc[2];
    } else if (name == "PITCH") {
      data_stream >> pitch;
    } else if (name == "YAW") {
      data_stream >> yaw;
    } else if (name == "FEED") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("feed", val));
    } else if (name == "RADIUS") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("radius", val));
    } else if (name == "CAPACITY") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("capacity", val));
    } else if (name == "CONTENTS") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("contents", val));
    } else {
    }
  }

  element->appendChild(vectorElement("location", loc[0], loc[1], loc[2],
                                     "in", "structural"));
  element->appendChild(vectorElement("orientation", "0", pitch, yaw,
                                     "rad", "body"));

  return element;
}

void
JSBImport::convertPropulsion(XMLElement* newTopElem, XMLElement* propElem)
{
  std::list<XMLElement::pointer>
    elems = propElem->getElements();
  std::list<XMLElement::pointer>::const_iterator
    it = elems.begin();
  while (it != elems.end()) {
    if ((*it)->getName() == "AC_ENGINE") {
      std::string engineFile = mEnginePath+(*it)->getAttribute("FILE")+".xml";
      std::ifstream infile;
      infile.open(engineFile.c_str());
      if (!infile.is_open())
        return;

      XMLDomParser parser;
      if (!parser.parseXML(infile))
        return;
      infile.close();

      shared_ptr<XMLDocument> doc = parser.getDocument();
      
      shared_ptr<XMLElement> propElem = parsePropulsion((*it)->getData());
      propElem->setName("engine");
//       propElem->appendChild(doc->getElement());
      
      newTopElem->appendChild(propElem);
    }
    else if ((*it)->getName() == "AC_TANK") {
    }
    else {
    }
    ++it;
  }
}

void
JSBImport::parseFCSComponent(const std::string& type, const std::string& name,
                             const std::string& data, XMLElement* fcs)
{
  if (type == "") {
    cerr << "No FCS TYPE attribute given in ??? FIXME" << endl;
    return;
  }

  std::stringstream data_stream(data);

  shared_ptr<XMLElement> element = new XMLElement("component");

  // Arrays where the transfer function rational is stored.
  std::vector<std::string> num;
  std::vector<std::string> denom;

  // Check if we need to figure out the default input value ...
  bool foundInput = false;

  if (type == "SUMMER") {
    element->appendChild(scalarElement("type", "summer"));
  } else if (type == "DEADBAND") {
    element->appendChild(scalarElement("type", "deadband"));
  } else if (type == "GRADIENT") {
    element->appendChild(scalarElement("type", "derivative"));
  } else if (type == "SWITCH") {
    element->appendChild(scalarElement("type", "switch"));
  } else if (type == "KINEMAT") {
    element->appendChild(scalarElement("type", "kinemat"));
  } else if (type == "PURE_GAIN" ||
             type == "SCHEDULED_GAIN" ||
             type == "AEROSURFACE_SCALE") {
    element->appendChild(scalarElement("type", "gain"));
  } else if (type == "INTEGRATOR") {
    element->appendChild(scalarElement("type", "integrator"));
  } else if (type == "LAG_FILTER") {
    element->appendChild(scalarElement("type", "transfer"));
    num.resize(2, "0");
    denom.resize(2, "0");
    denom[1] = "1";
  } else if (type == "LEAD_LAG_FILTER") {
    element->appendChild(scalarElement("type", "transfer"));
    num.resize(2, "0");
    denom.resize(2, "0");
  } else if (type == "SECOND_ORDER_FILTER") {
    element->appendChild(scalarElement("type", "transfer"));
    num.resize(3, "0");
    denom.resize(3, "0");
  } else if (type == "WASHOUT_FILTER") {
    element->appendChild(scalarElement("type", "transfer"));
    num.resize(2, "0");
    denom.resize(2, "0");
    num[1] = "1";
    denom[1] = "1";
  } else {
    cerr << "Unknown FCS COMPONENT type: \"" << type
         << "\". Ignoring whole FCS component \""
         << name << "\"!" << endl;
    return;
  }

  for (;;) {
    std::string token;
    data_stream >> token;
    if (!data_stream)
      break;

    if (token == "BIAS") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("bias", val));
      
    } else if (token == "CLIPTO") {
      std::string val[2];
      data_stream >> val[0] >> val[1];
      element->appendChild(scalarElement("limit-min", val[0]));
      element->appendChild(scalarElement("limit-max", val[1]));

    } else if (token == "C1") {
      std::string val;
      data_stream >> val;

      // What a mess ...
      if (type == "LAG_FILTER") {
        num[0] = val;
        denom[0] = val;
      } else if (type == "LEAD_LAG_FILTER") {
        num[1] = val;
      } else if (type == "SECOND_ORDER_FILTER") {
        num[2] = val;
      } else if (type == "WASHOUT_FILTER") {
        denom[0] = val;
      }
      
    } else if (token == "C2") {
      std::string val;
      data_stream >> val;

      if (type == "LEAD_LAG_FILTER") {
        num[0] = val;
      } else if (type == "SECOND_ORDER_FILTER") {
        num[1] = val;
      }
      
    } else if (token == "C3") {
      std::string val;
      data_stream >> val;

      if (type == "LEAD_LAG_FILTER") {
        denom[1] = val;
      } else if (type == "SECOND_ORDER_FILTER") {
        num[0] = val;
      }
      
    } else if (token == "C4") {
      std::string val;
      data_stream >> val;

      if (type == "LEAD_LAG_FILTER") {
        denom[0] = val;
      } else if (type == "SECOND_ORDER_FILTER") {
        denom[2] = val;
      }
      
    } else if (token == "C5") {
      std::string val;
      data_stream >> val;

      if (type == "SECOND_ORDER_FILTER") {
        denom[1] = val;
      }
      
    } else if (token == "C6") {
      std::string val;
      data_stream >> val;

      if (type == "SECOND_ORDER_FILTER") {
        denom[0] = val;
      }
      
    } else if (type == "KINEMAT" && token == "DETENTS") {
      int rows;
      data_stream >> rows;

      std::string table;
      do {
        std::string line;
        getline(data_stream, line, '\n');
        table += line + '\n';
      } while (0 <= --rows);

      element->appendChild(scalarElement("table", table));
      
    } else if (token == "GAIN") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("input-gain", val));
      
    } else if (token == "INPUT") {
      std::string val;
      data_stream >> val;
      if (0 < val.size() && val[0] == '-') {
        std::string rawInput = FCSName(val.substr(1));
        shared_ptr<XMLElement> inp = scalarElement("input", rawInput);
        shared_ptr<XMLElement> minus = new XMLElement("minus");
        minus->appendChild(inp);
        element->appendChild(minus);
      } else {
        element->appendChild(scalarElement("input", FCSName(val)));
      }
      foundInput = true;
      
    } else if (token == "INVERT") {
      // FIXME
      element->appendChild(scalarElement("input-gain", "-1"));
      
    } else if (token == "MAX") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("limit-max", val));
      
    } else if (token == "MIN") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("limit-min", val));
      
    } else if (type == "KINEMAT" && token == "NOSCALE") {
      element->appendChild(scalarElement("noscale", ""));
      
    } else if (token == "OUTPUT") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("output", FCSName(val)));

    } else if (token == "ROWS") {
      int rows;
      data_stream >> rows;

      std::string table;
      do {
        std::string line;
        getline(data_stream, line, '\n');
        table += line + '\n';
      } while (0 <= --rows);

      element->appendChild(scalarElement("table", table));
      
    } else if (token == "SCHEDULED_BY") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("lookup-input", val));
      
    } else if (type == "DEADBAND" && token == "WIDTH") {
      std::string val;
      data_stream >> val;
      element->appendChild(scalarElement("width", val));

    } else {
      cerr << "Unknown FCS COMPONENT keyword: \"" << token
           << "\". Ignoring whole FCS component \""
           << name << "\"!" << endl;
      return;
    }
  }

  // Handle the transfer function polynomials if there are any.
  if (0 < num.size()) {
    XMLElement* pelement = new XMLElement("numerator");
    // FIXME: add order to the attributes ...
    std::string poly;
    int idx = 0;
    while (idx < num.size()) {
      poly += num[idx++] + ' ';
    }
    pelement->setData(poly);
    element->appendChild(pelement);
  }
  if (0 < denom.size()) {
    XMLElement* pelement = new XMLElement("denominator");
    // FIXME: add order to the attributes ...
    std::string poly;
    int idx = 0;
    while (idx < denom.size()) {
      poly += denom[idx++] + ' ';
    }
    pelement->setData(poly);
    element->appendChild(pelement);
  }

  if (!foundInput) {
    if (mPrevFCSOut.size() <= 0) {
      // Emit a conversion error ...
      std::cerr << "Could not find default input for FCS component." << std::endl;
      return;
    }
    element->appendChild(scalarElement("input", mPrevFCSOut));
  }

  // JSBSim FCS components have a default output variable which could be
  // determined from its NAME attribute.
  mPrevFCSOut = FCSName(mFCSPrefix + normalizeName(name));
  element->appendChild(scalarElement("output", mPrevFCSOut));

  fcs->appendChild(element);
}

void
JSBImport::convertFCS(XMLElement* newFcElem, XMLElement* fcElem)
{
  if (!fcElem)
    return;

  // JSBSim has a very strange way to define default input values for FCS
  // components. The output of the prevous component is the default input
  // value. This one is stored here.
  mPrevFCSOut = std::string();

  
  std::list<XMLElement::pointer>
    comps = fcElem->getElements("COMPONENT");
  std::list<XMLElement::pointer>::const_iterator
    it = comps.begin();
  while (it != comps.end()) {
    parseFCSComponent((*it)->getAttribute("TYPE"),
                      (*it)->getAttribute("NAME"),
                      (*it)->getData(), newFcElem);
    ++it;
  }

  // Make shure it is not set when we exit here.
  mPrevFCSOut = std::string();
}

void
JSBImport::parseProductTable(XMLElement* prodElem,
                             const std::string& type, const std::string& data)
{
  std::vector<int> dims;
  std::vector<std::string> lookup_vars;
  if (type == "VECTOR")
    dims.resize(1), lookup_vars.resize(1);
  else if (type == "TABLE")
    dims.resize(2), lookup_vars.resize(2);
  else if (type == "TABLE3D")
    dims.resize(3), lookup_vars.resize(3);

  std::stringstream data_stream(data);
  std::string token;
  std::string line;
  // Throw away the first line. I don't know what this is for ...
  data_stream >> token;
  
  int idx;
  for (idx = 0; idx < dims.size(); ++idx) {
    data_stream >> dims[idx];
    if (!data_stream)
      return; //FIXME Error!!!
  }
  for (idx = 0; idx < lookup_vars.size(); ++idx) {
    data_stream >> lookup_vars[idx];
    if (!data_stream)
      return; //FIXME Error!!!
  }

  // Flush the rest of the line.
  getline(data_stream, line, '\n');
  // This time line will contain, the product values.
  getline(data_stream, line, '\n');

  // Replace the | characters in this line with blanks.
  std::string::size_type pos;
  while ((pos = line.find('|')) != std::string::npos)
    line.replace(pos, 1, " ");
  std::stringstream linestream(line);
  while (linestream >> token) {
    if (token.empty() || token == "none")
      break;

    prodElem->appendChild(scalarElement("property", token));
  }

  // In case of a constant ...
  if (lookup_vars.size() == 0) {
    data_stream >> token;
    prodElem->appendChild(scalarElement("constant", token));
    return;
  }

  // We have a real table ...
  shared_ptr<XMLElement> tableElem = new XMLElement("table");
  std::string dimension;
  dimension += '0' + lookup_vars.size();
  tableElem->setAttribute("dimension", dimension);

  for (idx = 0; idx < lookup_vars.size(); ++idx) {
    shared_ptr<XMLElement> inputElem = new XMLElement("input");
    std::string axis;
    if (lookup_vars.size() == 1)
      axis = "row";
    else if (lookup_vars.size() == 2) {
      if (idx == 0)
        axis = "column";
      else
        axis = "row";
    }
    else if (lookup_vars.size() == 3) {
      if (idx == 0)
        axis = "tab";
      else if (idx == 1)
        axis = "column";
      else
        axis = "row";
    }
    inputElem->setAttribute("axis", axis);
    inputElem->appendChild(scalarElement("property", lookup_vars[idx]));
    tableElem->appendChild(inputElem);
  }

  std::string tableData;
  getline(data_stream, tableData, '<');
  tableElem->appendChild(scalarElement("data", "\n" + tableData));
  
  prodElem->appendChild(tableElem);
}

void
JSBImport::convertAEROSummands(XMLElement* newAeroSummands, XMLElement* aeroSummands)
{
  shared_ptr<XMLElement> sumComp = new XMLElement("sum");

  std::list<XMLElement::pointer>
    elems = aeroSummands->getElements();
  std::list<XMLElement::pointer>::iterator
    it = elems.begin();
  while (it != elems.end()) {
    if ((*it)->getName() == "GROUP") {
      XMLElement* prodComp = new XMLElement("product");
      convertAEROSummands(prodComp, *it);
      sumComp->appendChild(prodComp);
    }
    else if ((*it)->getName() == "FACTOR") {
      parseProductTable(newAeroSummands, (*it)->getAttribute("TYPE"),
                        (*it)->getData());
    }
    else if ((*it)->getName() == "COEFFICIENT") {
      XMLElement* prodComp = new XMLElement("product");
      parseProductTable(prodComp, (*it)->getAttribute("TYPE"),
                        (*it)->getData());
      sumComp->appendChild(prodComp);
    }
    ++it;
  }

  newAeroSummands->appendChild(sumComp);
}

void
JSBImport::convertAERO(XMLElement* newAeroElem, XMLElement* aeroElem)
{
  if (!aeroElem)
    return;

  std::list<XMLElement::pointer>
    axes = aeroElem->getElements("AXIS");
  std::list<XMLElement::pointer>::iterator
    it = axes.begin();
  while (it != axes.end()) {
    if (!(*it))
      return; // FIXME

    // FIXME
    XMLElement* forceComp = new XMLElement("component");
    std::string name = (*it)->getAttribute("NAME");
    if (name == "LIFT")
      forceComp->setAttribute("direction", "L");
    else if (name == "DRAG")
      forceComp->setAttribute("direction", "D");
    else if (name == "SIDE")
      forceComp->setAttribute("direction", "C");
    else if (name == "ROLL")
      forceComp->setAttribute("direction", "L");
    else if (name == "PITCH")
      forceComp->setAttribute("direction", "M");
    else if (name == "YAW")
      forceComp->setAttribute("direction", "N");
    else
      ;

    convertAEROSummands(forceComp, *it);
    newAeroElem->appendChild(forceComp);

    ++it;
  }
}

shared_ptr<XMLDocument>
JSBImport::convert(shared_ptr<XMLDocument> jsbDoc)
{
  shared_ptr<XMLDocument> doc;

  if (!jsbDoc)
    return doc;

  shared_ptr<XMLElement> topElem = jsbDoc->getElement();
  if (!topElem)
    return doc;

  if (topElem->getName() != "FDM_CONFIG")
    return doc;

  if (topElem->getAttribute("VERSION").compare(0, 2, "1.") != 0)
    return doc;

  shared_ptr<XMLElement> newTopElem = new XMLElement("OpenFDM");
  newTopElem->setAttribute("version", "1");

  // Create the mechanics tag.
  shared_ptr<XMLElement> mechanicElem = new XMLElement("mechanical-system");
  // Add one masspoint to that mechanical system.
  // This one will represent the raw aircraft without additional fueltanks,
  // masspoints ...
  mRigidBody = new XMLElement("rigid-body");
  mechanicElem->appendChild(mRigidBody);

  // Parse the metrics section.
  shared_ptr<XMLElement> metricsElem = topElem->getElement("METRICS");
  if (!metricsElem)
    return doc;
  parseMetrics(metricsElem->getData(), newTopElem);

  // Parse the undercarriage section
  shared_ptr<XMLElement> undercarriageElem = topElem->getElement("UNDERCARRIAGE");
  if (undercarriageElem)
    parseUndercarriage(undercarriageElem->getData(), newTopElem);

  // Parse the propulsion section.
  shared_ptr<XMLElement> propulsionElem = topElem->getElement("PROPULSION");
  if (propulsionElem)
    convertPropulsion(newTopElem, propulsionElem);

  // Convert all the flight control system elements.
  XMLElement* newFcElem = new XMLElement("fcs");
  convertFCS(newFcElem, topElem->getElement("FLIGHT_CONTROL"));
  convertFCS(newFcElem, topElem->getElement("AUTOPILOT"));
  newTopElem->appendChild(newFcElem);

  // Now append the mechanical-system element.
  newTopElem->appendChild(mechanicElem);

  // Convert the aerodynamic force.
  XMLElement* newAeroElem = new XMLElement("force");
  newAeroElem->setAttribute("type", "aerodynamic");
  convertAERO(newAeroElem, topElem->getElement("AERODYNAMICS"));
  mRigidBody->appendChild(newAeroElem);

  doc = new XMLDocument;
  doc->setElement(newTopElem);
  return doc;
}

int main(int argc, char *argv[])
{
  if (argc < 3)
    return 1;

  JSBImport jsbImporter;
  
  jsbImporter.setAircraftPath(argv[1]);
  jsbImporter.setEnginePath(std::string(argv[1]) + "Engines/");
  jsbImporter.loadAircraft(argv[2]);
  
  shared_ptr<XMLDocument> own = jsbImporter.getConverted();
  
  if (own)
    std::cout << *own;

  return 0;
}
