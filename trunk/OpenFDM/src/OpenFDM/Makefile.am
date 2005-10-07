
INCLUDES = -I$(srcdir)/..

# check_PROGRAMS = main
# TESTS           = $(check_PROGRAMS)

noinst_PROGRAMS = main units

main_SOURCES = main.cpp
main_LDADD = libOpenFDM.la -lsgxml -lsgstructure

units_SOURCES = units.cpp
units_LDADD = libOpenFDM.la

OpenFDMincludedir = $(includedir)/OpenFDM

lib_LTLIBRARIES = libOpenFDM.la
nobase_OpenFDMinclude_HEADERS = \
  Matrix.h \
  Quaternion.h \
  Vector.h \
  OpenFDMConfig.h \
  Types.h \
LogStream.h \
Wind.h \
Plane.h \
Sensor.h \
Joint.h \
Transform.h \
Turbulence.h \
Rotation.h \
Inertia.h \
\
\
  AeroForce.h \
  Assert.h \
  Atmosphere.h \
  AtmosphereSTD1976.h \
  Bias.h \
  BinaryFunctionModel.h \
  Contact.h \
  ConstantForce.h \
  ConstSystem.h \
  ConstVisitor.h \
  DeadBand.h \
  DefaultGravity.h \
  DefaultGround.h \
  DefaultPlanet.h \
  DirectForce.h \
  DiscreteIntegrator.h \
  DoPri5.h \
  Environment.h \
  EnvironmentObject.h \
  ExplicitAdams.h \
  ExplicitEuler.h \
  Expression.h \
  Force.h \
  Frame.h \
  FreeJoint.h \
  Function.h \
  Gain.h \
  Gravity.h \
  Ground.h \
  Input.h \
  Integrator.h \
  ImplicitEuler.h \
  JointT.h \
  Limits.h \
  LinAlg/Algorithm.h \
  LinAlg/Array.h \
  LinAlg/Base.h \
  LinAlg/Container.h \
  LinAlg/Expressions.h \
  LinAlg/IO.h \
  LinAlg/Misc.h \
  LinAlg/Proxy.h \
  Mass.h \
  Math.h \
  MidpointRule.h \
  Model.h \
  ModelGroup.h \
  MultiBodyModel.h \
  Newton.h \
  Object.h \
  ODESolver.h \
  Planet.h \
  Property.h \
  Product.h \
  PrismaticJoint.h \
  ReaderWriter.h \
  Referenced.h \
  RefPtr.h \
  RevoluteActuator.h \
  RevoluteJoint.h \
  RigidBody.h \
  RootFrame.h \
  Saturation.h \
  SimpleContact.h \
  SimpleGear.h \
  Summer.h \
  System.h \
  Table.h \
  TimeDerivative.h \
  TransferFunction.h \
  UnaryFunctionModel.h \
  Units.h \
  Variant.h \
  Vehicle.h \
  Visitor.h \
  XML/Tablereader.h \
  XML/XMLReader.h

libOpenFDM_la_SOURCES = \
  AeroForce.cpp \
  Assert.cpp \
  Atmosphere.cpp \
  AtmosphereSTD1976.cpp \
  Bias.cpp \
  BinaryFunctionModel.cpp \
  Contact.cpp \
  ConstantForce.cpp \
  ConstSystem.cpp \
  DeadBand.cpp \
  DefaultGravity.cpp \
  DefaultGround.cpp \
  DefaultPlanet.cpp \
  DirectForce.cpp \
  DiscreteIntegrator.cpp \
  DoPri5.cpp \
  Environment.cpp \
  EnvironmentObject.cpp \
  ExplicitAdams.cpp \
  ExplicitEuler.cpp \
  Expression.cpp \
  Force.cpp \
  Frame.cpp \
  FreeJoint.cpp \
  Function.cpp \
  Gain.cpp \
  Gravity.cpp \
  Ground.cpp \
  Input.cpp \
  Integrator.cpp \
  ImplicitEuler.cpp \
  Logger.cpp \
  Mass.cpp \
  MidpointRule.cpp \
  Model.cpp \
  ModelGroup.cpp \
  MultiBodyModel.cpp \
  Newton.cpp \
  Object.cpp \
  ODESolver.cpp \
  Planet.cpp \
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
  TimeDerivative.cpp \
  TransferFunction.cpp \
  UnaryFunctionModel.cpp \
  Units.cpp \
  Variant.cpp \
  Vehicle.cpp \
  XML/Tablereader.cpp \
  XML/XMLReader.cpp
