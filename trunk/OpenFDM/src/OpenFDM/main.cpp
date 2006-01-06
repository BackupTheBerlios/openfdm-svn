/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <list>
#include <string>
using namespace std;

#include <OpenFDM/Object.h>
#include <OpenFDM/Vector.h>
#include <OpenFDM/Matrix.h>
#include <OpenFDM/Quaternion.h>
#include <OpenFDM/Frame.h>
#include <OpenFDM/Friction2D.h>
#include <OpenFDM/Units.h>
#include <OpenFDM/Planet.h>
#include <OpenFDM/DefaultPlanet.h>
#include <OpenFDM/RigidBody.h>
#include <OpenFDM/MobileRootJoint.h>
#include <OpenFDM/AeroForce.h>
#include <OpenFDM/SimpleContact.h>
#include <OpenFDM/SimpleGear.h>
#include <OpenFDM/ExplicitEuler.h>
#include <OpenFDM/ExplicitAdams.h>
#include <OpenFDM/ImplicitEuler.h>
#include <OpenFDM/MidpointRule.h>
#include <OpenFDM/DoPri5.h>
#include <OpenFDM/Newton.h>
#include <OpenFDM/Mass.h>
#include <OpenFDM/Vehicle.h>
#include <OpenFDM/RevoluteJoint.h>
#include <OpenFDM/PrismaticJoint.h>
#include <OpenFDM/ReaderWriter.h>
#include <OpenFDM/LogStream.h>
#include <OpenFDM/Variant.h>
#include <OpenFDM/Property.h>

// Model includes
#include <OpenFDM/Model.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/Bias.h>
#include <OpenFDM/ConstModel.h>
#include <OpenFDM/DeadBand.h>
#include <OpenFDM/DiscreteIntegrator.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/TimeDerivative.h>
#include <OpenFDM/UnaryFunctionModel.h>
#include <OpenFDM/BinaryFunctionModel.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Summer.h>

#include "XML/XMLReader.h"

using namespace OpenFDM;

namespace OpenFDM {

class OpenFDMReader :
    public ReaderWriter {
public:
  OpenFDMReader(void) {}
  virtual ~OpenFDMReader(void) {}

//   void setInputFile(const std::string& inputFile)
//   { mInputFile = inputFile; }
//   const std::string& getInputFile(void) const
//   { return mInputFile; }

//   // FIXME
//   std::string mInputFile;


  // FIXME
  virtual void reset(void)
  {}

  /** Read scalar value element from dom tree.
   * 
   */
  template<typename T>
  bool
  readScalar(const XMLElement* element, T& scalar)
  {
    if (!element)
      return error("Error reading scalar: no such element!");
    
    // FIXME handle units!!!

    std::stringstream stream(element->getData());
    stream >> scalar;
    if (stream.fail())
      return error("Error reading scalar: Could not read scalar data!");

    return true;
  }
  
  /** Read scalar value element from dom tree.
   * 
   */
  bool
  readVector3(const XMLElement* element, Vector3& v)
  {
    if (!element || element->getName() != "Vector")
      return error("Error reading Vector: no such element!");
    
    // FIXME handle units!!!
    std::stringstream stream(element->getData());
    stream >> v(1) >> v(2) >> v(3);
    if (stream.fail())
      return error("Error reading Vector: Could not read vector data!");

    return true;
  }
  
  bool
  readQuaternion(const XMLElement* element, Quaternion& q)
  {
    if (!element || element->getName() != "quaternion")
      return error("Error reading Quternion: no such element!");
    
    std::stringstream stream(element->getData());
    stream >> q(1) >> q(2) >> q(3) >> q(4);
    if (stream.fail())
      return error("Error reading Quaternion: Could not read vector data!");
    
    // Check if the norm is properly normalized
    bool normalized = (norm(q)-1) < 8*Limits<Quaternion::value_type>::epsilon();
    if (!normalized)
      return error("Error reading Quaternion: Quaternion is not normalized!");

    return true;
  }

  bool
  readMatrix(const XMLElement* element, Matrix& m)
  {
    if (!element || element->getName() != "matrix")
      return error("Error reading Vector: no such element!");
    
    unsigned columns = 1;
    std::string columnsStr = element->getAttribute("columns");
    if (!columnsStr.empty()) {
      std::stringstream stream(columnsStr);
      stream >> columns;
      if (stream.fail())
        error("Error reading matrix: cannot read Columns attribute!");
    }
    
    unsigned rows = 1;
    std::string rowsStr = element->getAttribute("rows");
    if (!rowsStr.empty()) {
      std::stringstream stream(rowsStr);
      stream >> rows;
      if (stream.fail())
        error("Error Reading Matrix: cannot read Rows attribute!");
    }

    m.resize(rows, columns);
    // FIXME handle units!!!
    std::stringstream stream(element->getData());
    for (unsigned i = 1; i <= rows; ++i)
      for (unsigned j = 1; j <= columns; ++j)
        stream >> m(i, j);

    if (stream.fail())
      return error("Error reading Matrix: Could not read matrix data!");

    return true;
  }

  bool
  readOrientation(const XMLElement* element, Quaternion& q)
  {
    if (!element || element->getName() != "Orientation")
      return error("Error reading Orientation: no such element!");
    
    // Check if the orientation is present in a quaternion.
    std::list<XMLElement::const_pointer> qs = element->getElements("quaternion");
    // Fail if there are multiple elements.
    if (1 < qs.size())
      return error("Error reading Orientation: Multiple Quaternion elements!");
    if (1 == qs.size()) {
      if (!readQuaternion(qs.front(), q))
        return error("Error reading Orientation: Could not read Quaternion!");
      else
        return true;
    }
    
    // Check if the orientation is present in angle axis representation.
    std::list<XMLElement::const_pointer> angs = element->getElements("Angle");
    std::list<XMLElement::const_pointer> axs = element->getElements("Axis");
    if (0 < angs.size() || 0 < axs.size()) {
      // Fail if there are multiple elements.
      if (1 < angs.size())
        return error("Error reading Orientation: Multiple Angle elements!");
      if (0 == angs.size())
        return error("Error reading Orientation: No Angle element where Axis element is present!");
      if (1 < axs.size())
        return error("Error reading Orientation: Multiple Axis elements!");
      if (0 == axs.size())
        return error("Error reading Orientation: No Axis element where Angle is present!");
      
      // Read angle.
      real_type angle;
      if (!readScalar(angs.front(), angle))
        return error("Error reading Orientation: Could not read Angle!");
      
      // Read axis.
      Vector3 axis;
      if (!readVector3(axs.front(), axis))
        return error("Error reading Orientation: Could not read Axis!");
      
      q = Quaternion::fromAngleAxis(angle, axis);
      return true;
    }
    
    // Check if the orientation is present in euler angle representation.
    std::list<XMLElement::const_pointer> rll = element->getElements("Roll");
    std::list<XMLElement::const_pointer> att = element->getElements("Attitude");
    std::list<XMLElement::const_pointer> hdng = element->getElements("Heading");
    if (0 < rll.size() || 0 < att.size() || 0 < hdng.size()) {
      // Fail if there are multiple elements.
      if (1 < rll.size())
        return error("Error reading Orientation: Multiple Roll elements!");
      if (1 < att.size())
        return error("Error reading Orientation: Multiple Attitude elements!");
      if (1 < hdng.size())
        return error("Error reading Orientation: Multiple Heading elements!");
      
      // Read the roll angle.
      real_type roll = 0;
      if (0 < rll.size() && !readScalar(rll.front(), roll))
        return error("Error reading Orientation: Could not read Roll!");
      
      // Read the pitch angle.
      real_type attitude = 0;
      if (0 < att.size() && !readScalar(att.front(), attitude))
        return error("Error reading Orientation: Could not read Attitude!");
      
      // Read the heading angle.
      real_type heading = 0;
      if (0 < hdng.size() && !readScalar(hdng.front(), heading))
        return error("Error reading Orientation: Could not read Heading!");
      
      // FIXME: verify ...
      q = Quaternion::fromHeadAttBank(attitude, heading, roll);
      
      return true;
    }
    
    return error("Error reading Orientation: No Orientation data found!");
  }

  bool
  readPort(const XMLElement* element, std::string& name, unsigned& number)
  {
    name = element->getAttribute("name");
    if (name.empty())
      return error("Port found without or with empty name attribute!");
    if (!readScalar(element, number))
      return error("Can not read port number!");
    return true;
  }

  bool
  readObjectProperties(const XMLElement* element, Object* object)
  {
    std::list<std::string> props = object->listProperties();
    std::list<std::string>::iterator it = props.begin();
    while (it != props.end()) {
      if (1 < element->getNumElements(*it))
        return error(std::string("Error loading Model: More than one \"")
                     + *it + "\" Property tag in model definition!");

      const XMLElement* propElement = element->getElement(*it);
      if (propElement) {
        if (1 != propElement->getNumElements("matrix"))
          return error("InitialValue tag found without content!");

        Matrix m;
        if (!readMatrix(propElement->getElement("matrix"), m))
          return error("Cannot read InitialValue Matrix!");

        object->setPropertyValue(*it, Variant(m));

        // FIXME add more here
      }
      ++it;
    }
  }

  bool
  readModel(const XMLElement* modelElement, ModelGroup* group)
  {
   std::string name = modelElement->getAttribute("name");
    if (name.empty())
      return error("Error loading Models: Empty name!");

    std::string type = modelElement->getAttribute("type");
    SharedPtr<Model> model;
    if (type == "Bias") {
      model = new Bias(name);
    } else if (type == "Const") {
      model = new ConstModel(name, Matrix()); // FIXME
    } else if (type == "DeadBand") {
      model = new DeadBand(name);
    } else if (type == "DiscreteIntegrator") {
      model = new DiscreteIntegrator(name);
    } else if (type == "Gain") {
      model = new Gain(name);
    } else if (type == "Integrator") {
      model = new Integrator(name);
    } else if (type == "Product") {
      model = new Product(name);
    } else if (type == "Saturation") {
      model = new Saturation(name);
    } else if (type == "TimeDerivative") {
      model = new TimeDerivative(name);
    } else
      return error(std::string("Error loading Models: Unknown Model type \"")
                   + type + "\" !");

    if (!readObjectProperties(modelElement, model))
      return error("Error loading Models: Error reading properties !");

    group->addModel(model);

    return true;
  }

  bool
  readModelGroup(const XMLElement* controlSystem, ModelGroup* group)
  {
    // Read all Models.
    std::list<XMLElement::const_pointer> models
      = controlSystem->getElements("model");
    if (0 == models.size())
      return error("Error loading Vehicle: No Model found");

    std::list<XMLElement::const_pointer>::iterator it = models.begin();
    while (it != models.end()) {
      if (!readModel(*it, group))
        return false;
      ++it;
    }

    std::list<XMLElement::const_pointer> connections
      = controlSystem->getElements("connect");
    it = connections.begin();
    while (it != connections.end()) {
      if (1 != (*it)->getNumElements("src"))
        return error("No src element found in connect element!");
      std::string name;
      unsigned number;
      if (!readPort((*it)->getElement("src"), name, number))
        return false;

      Model* srcModel = group->getModel(name);
      if (!srcModel)
        return error("Can not find src model named \"" + name + "\" !");
      if (srcModel->getNumOutputPorts() <= number)
        return error("Model \"" + name + "\" has not enough output ports!");
      Port* srcPort = srcModel->getOutputPort(number);
      if (!srcPort)
        return error("Model \"" + name + "\" returns invalid output port!");

      if (1 != (*it)->getNumElements("dst"))
        return error("No dst element found in connect element!");
      if (!readPort((*it)->getElement("dst"), name, number))
        return false;

      Model* dstModel = group->getModel(name);
      if (!dstModel)
        return error("Can not find dst model named \"" + name + "\" !");
      if (dstModel->getNumInputPorts() <= number)
        return error("Model \"" + name + "\" has not enough input ports!");

      Port* dstPort = dstModel->getInputPort(number);
      if (!dstPort)
        return error("Model \"" + name + "\" returns invalid input port!");
      dstPort->connect(srcPort);

      // FIXME
      ++it;
    }

    return true;
  }

  bool
  readRigidBodies(const XMLElement* mechanicalSystem)
  {
    // Read all rigid bodies.
    std::list<XMLElement::const_pointer> rigidBodies
      = mechanicalSystem->getElements("RigidBody");
    if (0 == rigidBodies.size())
      return error("Error loading Vehicle: No RigidBody found");

    std::list<XMLElement::const_pointer>::iterator it = rigidBodies.begin();
    while (it != rigidBodies.end()) {
      RigidBody* rigidBody = new RigidBody("rigid body");
      rigidBody->setName((*it)->getAttribute("Name"));
      
//       if (mVehicle->getRigidBody((*it)->getAttribute("Name")))
//         return error(std::string("Error loading RigidBodies: Duplicate Rigid Body Name \"")
//                      + (*it)->getAttribute("Name") + "\" !");
      
//       mVehicle->addRigidBody(rigidBody);
      ++it;
    }
    return true;
  }

  bool
  readJoints(const XMLElement* mechanicalSystem)
  {
    // Read all joints bodies.
    std::list<XMLElement::const_pointer> joints
      = mechanicalSystem->getElements("Joint");
    std::list<XMLElement::const_pointer>::iterator it = joints.begin();
    while (it != joints.end()) {
      Joint* joint = 0;
      std::string name = (*it)->getAttribute("Name");
      if ((*it)->getAttribute("Type") == "Revolute")
        joint = new RevoluteJoint(name);
      else if ((*it)->getAttribute("Type") == "Prismatic")
        joint = new PrismaticJoint(name);
      if (!joint)
        return error(std::string("Error loading Vehicle: Unknown Joint type \"")
                     + (*it)->getAttribute("Type") + "\" !");
      
      
//       if (mVehicle->getJoint((*it)->getAttribute("Name")))
//         return error(std::string("Error loading Vehicle: Duplicate Name \"")
//                      + (*it)->getAttribute("Name") + "\" !");
//       mVehicle->addJoint(joint);
      
      
      // Connect the joints to the rigid bodies.
      std::list<XMLElement::const_pointer> mounts
        = (*it)->getElements("Mount");
      if (mounts.size() != 2)
        return error("Error loading Vehicle: More or less then 2 Mounts!");
      std::list<XMLElement::const_pointer>::iterator mit = mounts.begin();
      for (unsigned idx = 0; mit != mounts.end(); ++idx, ++mit) {
        std::string mountAttrName = (*mit)->getAttribute("Name");
//         RigidBody* body = mVehicle->getRigidBody(mountAttrName);
//         if (!body)
//           return error(std::string("Error loading Vehicle:") +
//                        " Can not find RigidBody \"" + mountAttrName +
//                        "\" for Mount of Joint \"" + joint->getName() + "\"!");
       
//         joint->setParentFrame(body, idx);
        
        // Read the orientation. No orientation element means unit orientation.
        // Multiple orientations is an error.
        std::list<XMLElement::const_pointer> orients
          = (*mit)->getElements("Orientation");
        Quaternion orientation = Quaternion::unit();
        if (0 < orients.size()) {
          if (1 < orients.size())
            return error("Error loading Vehicle: More than one Orientation!");
          if (!readOrientation(orients.front(), orientation))
            return error("Error loading Vehicle: Could not read Orientation!");
        }
        
        // Read the position. No position element means unit position.
        // Multiple positions is an error.
        std::list<XMLElement::const_pointer> positions
          = (*mit)->getElements("Position");
        Vector3 position = Vector3::zeros();
        if (0 < positions.size()) {
          if (1 < positions.size())
            return error("Error loading Vehicle: More than one Position!");
          
          std::list<XMLElement::const_pointer> vectors
            = positions.front()->getElements("Vector");
          if (vectors.size() != 1)
            return error("Error loading Vehicle: More than 1 Vector!");
          if (!readVector3(vectors.front(), position))
            return error("Error loading Vehicle: Could not read Position!");
        }
        
      }
      ++it;
    }

    return true;
  }

  bool
  loadVehicle(const std::string& acFile)
  {
    std::ifstream infile;
    infile.open(acFile.c_str());
    if (!infile.is_open())
      return 0;
    
    XMLDomParser parser;
    if (!parser.parseXML(infile))
      return error("Error loading Vehicle: Could not parse xml data!");
    
    infile.close();
    
    XMLDocument* doc = parser.getDocument();
    if (!doc)
      return error("Error loading Vehicle: Could not parse xml data!");
    XMLElement* openFDM = doc->getElement();
    if (!openFDM)
      return error("Error loading Vehicle: Could not get toplevel element!");
    if (openFDM->getName() != "OpenFDM")
      return error("Error loading Vehicle: Toplevel element is not an "
                   "OpenFDM element!");
    if (openFDM->getAttribute("version") != "1")
      return error("Error loading Vehicle: Invalid OpenFDM version!");
    
    XMLElement* mechanicalSystem = openFDM->getElement("MechanicalSystem");
    if (!mechanicalSystem)
      return error("Error loading Vehicle: Cannot find a MechanicalSystem "
                   "element!");
    
    XMLElement* controlSystem = openFDM->getElement("ControlSystem");
    if (!controlSystem)
      return error("Error loading Vehicle: Cannot find a MechanicalSystem "
                   "element!");


    mVehicle = new Vehicle;
    
    if (!readModelGroup(controlSystem, mVehicle->getModelGroup()))
      return error("Error loading Vehicle: Can not load Model elements!");

    if (!readRigidBodies(mechanicalSystem))
      return error("Error loading Vehicle: Can not load RigidBody elements!");
    
    if (!readJoints(mechanicalSystem))
      return error("Error loading Vehicle: Can not load Joint elements!");

    mVehicle->setGeodPosition(Geodetic(0.0, 0.0, 0.0));

    return true;
  }
};
  
} // namespace OpenFDM

void printVehicle(Vehicle* vehicle)
{
  cout << "T = " << vehicle->getTime()
       << ", Pos: " << vehicle->getGeodPosition()
//        << ", Or: " << vehicle->getGeodOrientation()
       << endl;

}

int main(int argc, char *argv[])
{
#if 1
  if (argc < 2)
    return 1;
  
  OpenFDMReader reader;
  reader.loadVehicle(argv[1]);
  SharedPtr<Vehicle> vehicle = reader.getVehicle();

  if (reader.getErrorState()) {
    std::cerr << "Could not read vehicle:" << std::endl;
    const ReaderWriter::StringList errors = reader.getErrors();
    ReaderWriter::StringList::const_iterator it = errors.begin();
    while (it != errors.end()) {
      std::cerr << *it << std::endl;
      ++it;
    }
    return 1;
  }

  if (!vehicle) {
    std::cerr << "could not read vehicle" << std::endl;
    return 1;
  }

  printVehicle(vehicle);

  System* system = vehicle->getSystem();
  bool initOk = system->init();
  if (!initOk) {
    cout << "Error  in init" << endl;
    return -17;
  }
  ModelGroup* modelGroup = vehicle->getModelGroup();
  for (unsigned i = 0; i < modelGroup->getNumModels(); ++i) {
    cout << modelGroup->getModel(i)->getName() << endl;
  }
  for (unsigned j = 0; j < 1000; ++j) {
    system->simulate(j*0.01);

    for (unsigned i = 0; i < modelGroup->getNumModels(); ++i)
      cout << modelGroup->getModel(i)->getOutputPort("output")->getValue().toMatrix() << " ";

    cout << endl;
  }

#else
  SharedPtr<Vehicle> vehicle = new Vehicle();

  Geodetic geod;
  geod.longitude = -81.636*deg2rad;
  geod.latitude = 28.594*deg2rad;
  geod.altitude = convertFrom(uFoot, 918720.0);
  vehicle->setGeodPosition(geod);

  real_type heading = 90*deg2rad;
  Quaternion q = Quaternion::fromAngleAxis(heading, Vector3::unit(3));
  vehicle->setGeodOrientation(q);

  vehicle->getMobileRootJoint()->setRelVel(Vector6::zeros());
  Vector3 vel(convertFrom(uFeetPSecond, 23900.0), 0, 0);
  vehicle->getMobileRootJoint()->setLinearRelVel(vel);
  vehicle->getTopBody()->addMultiBodyModel(new Mass("Testmass", SpatialInertia(InertiaMatrix(100,0,0,100,0,100), 100), "Mass"));


  RigidBody* body = new RigidBody;
  body->setName("Body am revolute Joint");
  body->addMultiBodyModel(new Mass("Testmass", SpatialInertia(InertiaMatrix(1,0,0,1,0,1), 1), "Mass"));
  vehicle->getTopBody()->addChildFrame(body);
  PrismaticJoint* joint = new PrismaticJoint("Prismatic Joint 1");
  // FIXME, seem to have a poblem with the 2*pi periodicity,
  // use some type of quaternion ...
//   RevoluteJoint* joint = new RevoluteJoint("Revolute Joint 1");
  joint->setJointAxis(Vector3(1, 0, 0));
  vehicle->getTopBody()->addMultiBodyModel(joint);
  body->addMultiBodyModel(joint, 1);


  RigidBody* body2 = new RigidBody;
  body2->setName("Body2 am prismatic Joint");
  body2->addMultiBodyModel(new Mass("Testmass", SpatialInertia(InertiaMatrix(1,0,0,1,0,1), 1), "Mass"));
  body->addChildFrame(body2);
//   PrismaticJoint* joint2 = new PrismaticJoint("Prismatic Joint 1");
  RevoluteJoint* joint2 = new RevoluteJoint("Revolute Joint 2");
  joint2->setJointAxis(Vector3(0, 1, 0));
  body->addMultiBodyModel(joint2);
  body2->addMultiBodyModel(joint2, 1);

  vehicle->init();

  printVehicle(vehicle);


  real_type tEnd = 200;
  real_type dt = 10;
//   dt = 1.0/128;
  while (vehicle->getTime() < tEnd) {
    vehicle->output();
    vehicle->update(dt);
    printVehicle(vehicle);
  }
#endif

  return 0;
}

