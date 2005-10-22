
INCLUDES = -I$(srcdir)/..

# check_PROGRAMS = main
# TESTS           = $(check_PROGRAMS)

noinst_PROGRAMS = main units

main_SOURCES = main.cpp
main_LDADD = libOpenFDM.la

units_SOURCES = units.cpp
units_LDADD = libOpenFDM.la

OpenFDMincludedir = $(includedir)/OpenFDM

lib_LTLIBRARIES = libOpenFDM.la

# Will need these at the moment ...
libOpenFDM_la_LIBADD = -lsgxml -lsgstructure
nobase_OpenFDMinclude_HEADERS = \
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
  Inertia.h \
  Joint.h \
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
  LogStream.h \
  Mass.h \
  Math.h \
  Matrix.h \
  MidpointRule.h \
  Model.h \
  ModelGroup.h \
  MultiBodyModel.h \
  Newton.h \
  Object.h \
  ODESolver.h \
  OpenFDMConfig.h \
  Plane.h \
  Planet.h \
  Property.h \
  Product.h \
  PrismaticJoint.h \
  Quaternion.h \
  ReaderWriter.h \
  Referenced.h \
  RefPtr.h \
  RevoluteActuator.h \
  RevoluteJoint.h \
  RigidBody.h \
  Rotation.h \
  RootFrame.h \
  Saturation.h \
  Sensor.h \
  SimpleContact.h \
  SimpleGear.h \
  Summer.h \
  System.h \
  Table.h \
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
  Visitor.h \
  Wind.h \
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
