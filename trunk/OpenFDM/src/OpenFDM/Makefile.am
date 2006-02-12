SUBDIRS = XML

INCLUDES = -I$(srcdir)/..

noinst_PROGRAMS = units

# define some unit tests
check_PROGRAMS = quattest
TESTS          = $(check_PROGRAMS)

units_SOURCES = units.cpp
units_LDADD = libOpenFDM.la

quattest_SOURCES = quattest.cpp
quattest_LDADD = Units.o
#quattest_LDADD = libOpenFDM.la

OpenFDMincludedir = $(includedir)/OpenFDM

lib_LTLIBRARIES = libOpenFDM.la

# Will need these at the moment ...
nobase_OpenFDMinclude_HEADERS = \
  AeroForce.h \
  AirSpring.h \
  Assert.h \
  Atmosphere.h \
  AtmosphereSTD1976.h \
  Bias.h \
  BinaryFunctionModel.h \
  CartesianActuatorFrame.h \
  CartesianJointFrame.h \
  Contact.h \
  ConstModel.h \
  ConstFrameVisitor.h \
  CowPtr.h \
  DeadBand.h \
  DefaultGravity.h \
  DefaultGround.h \
  DefaultPlanet.h \
  DiscBrake.h \
  DiscreteIntegrator.h \
  DoPri5.h \
  Environment.h \
  EnvironmentObject.h \
  ExplicitAdams.h \
  ExplicitEuler.h \
  ExternalForceModel.h \
  Force.h \
  Frame.h \
  FrameVisitor.h \
  Function.h \
  Gain.h \
  Gravity.h \
  Ground.h \
  Input.h \
  Integrator.h \
  Interact.h \
  ImplicitEuler.h \
  Inertia.h \
  Joint.h \
  Launchbar.h \
  Limits.h \
  LinAlg/Algorithm.h \
  LinAlg/Array.h \
  LinAlg/Base.h \
  LinAlg/Container.h \
  LinAlg/Expressions.h \
  LinAlg/IO.h \
  LinAlg/Misc.h \
  LinAlg/Proxy.h \
  LinearSpringDamper.h \
  LogStream.h \
  Mass.h \
  Math.h \
  Matrix.h \
  MaxModel.h \
  MidpointRule.h \
  MinModel.h \
  MobileRootJoint.h \
  MobileRootJointFrame.h \
  Model.h \
  ModelGroup.h \
  ModelVisitor.h \
  MultiBodySystem.h \
  Newton.h \
  Object.h \
  ODESolver.h \
  OpenFDMConfig.h \
  Plane.h \
  Planet.h \
  Port.h \
  Property.h \
  Product.h \
  PrismaticJoint.h \
  PrismaticJointFrame.h \
  Quaternion.h \
  ReaderWriter.h \
  Referenced.h \
  RevoluteActuator.h \
  RevoluteActuatorFrame.h \
  RevoluteJoint.h \
  RevoluteJointFrame.h \
  RigidBody.h \
  Rotation.h \
  RootFrame.h \
  SampleTime.h \
  Saturation.h \
  SharedPtr.h \
  Sensor.h \
  SimpleContact.h \
  SimpleGear.h \
  StateStream.h \
  Summer.h \
  System.h \
  Table.h \
  TableData.h \
  Tank.h \
  TaskInfo.h \
  TimeDerivative.h \
  TransferFunction.h \
  Transform.h \
  Turbulence.h \
  Types.h \
  UnaryFunctionModel.h \
  Units.h \
  Variant.h \
  Vector.h \
  Vehicle.h \
  WeakPtr.h \
  WeakReferenced.h \
  WheelContact.h \
  Wind.h \
  XMLDumpModelVisitor.h

libOpenFDM_la_SOURCES = \
  AeroForce.cpp \
  AirSpring.cpp \
  Assert.cpp \
  Atmosphere.cpp \
  AtmosphereSTD1976.cpp \
  Bias.cpp \
  BinaryFunctionModel.cpp \
  Contact.cpp \
  ConstModel.cpp \
  DeadBand.cpp \
  DefaultGravity.cpp \
  DefaultGround.cpp \
  DefaultPlanet.cpp \
  DiscBrake.cpp \
  DiscreteIntegrator.cpp \
  DoPri5.cpp \
  Environment.cpp \
  EnvironmentObject.cpp \
  ExplicitAdams.cpp \
  ExplicitEuler.cpp \
  ExternalForceModel.cpp \
  Force.cpp \
  Frame.cpp \
  Function.cpp \
  Gain.cpp \
  Gravity.cpp \
  Ground.cpp \
  Input.cpp \
  Integrator.cpp \
  Interact.cpp \
  ImplicitEuler.cpp \
  Joint.cpp \
  Launchbar.cpp \
  LinearSpringDamper.cpp \
  Logger.cpp \
  Mass.cpp \
  MaxModel.cpp \
  MidpointRule.cpp \
  MinModel.cpp \
  MobileRootJoint.cpp \
  Model.cpp \
  ModelGroup.cpp \
  MultiBodySystem.cpp \
  Newton.cpp \
  Object.cpp \
  ODESolver.cpp \
  Planet.cpp \
  Port.cpp \
  PrismaticJoint.cpp \
  Product.cpp \
  Property.cpp \
  ReaderWriter.cpp \
  RevoluteActuator.cpp \
  RevoluteJoint.cpp \
  RigidBody.cpp \
  RootFrame.cpp \
  Saturation.cpp \
  SimpleContact.cpp \
  SimpleGear.cpp \
  Summer.cpp \
  System.cpp \
  Table.cpp \
  Tank.cpp \
  TimeDerivative.cpp \
  TransferFunction.cpp \
  UnaryFunctionModel.cpp \
  Units.cpp \
  Variant.cpp \
  Vehicle.cpp \
  WheelContact.cpp
