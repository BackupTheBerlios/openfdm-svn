SUBDIRS = XML

INCLUDES = -I$(srcdir)/..

OpenFDMincludedir = $(includedir)/OpenFDM

lib_LTLIBRARIES = libOpenFDM.la
libOpenFDM_la_LIBADD = -lrt

OpenFDMCoreHEADERS = \
  Assert.h \
  Atomic.h \
  Connection.h \
  ConstFrameVisitor.h \
  CowPtr.h \
  DoPri5.h \
  Environment.h \
  EnvironmentObject.h \
  ExplicitAdams.h \
  ExplicitEuler.h \
  Frame.h \
  FrameVisitor.h \
  Function.h \
  GroupInput.h \
  GroupOutput.h \
  ImplicitEuler.h \
  Inertia.h \
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
  Newton.h \
  MidpointRule.h \
  Model.h \
  ModelGroup.h \
  ModelVisitor.h \
  Mutex.h \
  MPFRFloat.h \
  NumericPortAcceptor.h \
  NumericPortProvider.h \
  NumericPortProxy.h \
  Object.h \
  ODESolver.h \
  OpenFDMConfig.h \
  PhysicalDimension.h \
  Plane.h \
  Planet.h \
  Port.h \
  PortAcceptor.h \
  PortProvider.h \
  Quaternion.h \
  ReaderWriter.h \
  Referenced.h \
  Rotation.h \
  SampleTime.h \
  ScopeLock.h \
  SharedPtr.h \
  SimulationTime.h \
  StateStream.h \
  System.h \
  TableData.h \
  TaskInfo.h \
  Time.h \
  Transform.h \
  TypeInfo.h \
  Types.h \
  TypeTraits.h \
  Unit.h \
  Variant.h \
  Vector.h \
  WeakPtr.h \
  WeakReferenced.h \
  XMLDumpModelVisitor.h

OpenFDMCoreSOURCES = \
  Assert.cpp \
  Atomic.cpp \
  Connection.cpp \
  DoPri5.cpp \
  Environment.cpp \
  EnvironmentObject.cpp \
  ExplicitAdams.cpp \
  ExplicitEuler.cpp \
  Frame.cpp \
  Function.cpp \
  GroupInput.cpp \
  GroupOutput.cpp \
  ImplicitEuler.cpp \
  Logger.cpp \
  Mass.cpp \
  Newton.cpp \
  MidpointRule.cpp \
  Model.cpp \
  ModelGroup.cpp \
  Object.cpp \
  ODESolver.cpp \
  Planet.cpp \
  Port.cpp \
  PortAcceptor.cpp \
  PortProvider.cpp \
  ReaderWriter.cpp \
  SampleTime.cpp \
  SimulationTime.cpp \
  System.cpp \
  TaskInfo.cpp \
  Time.cpp \
  Unit.cpp \
  Variant.cpp

OpenFDMModelsHEADERS = \
  AeroForce.h \
  AirSpring.h \
  Atmosphere.h \
  AtmosphereSTD1976.h \
  Bias.h \
  BinaryFunctionModel.h \
  ConstModel.h \
  Contact.h \
  DeadBand.h \
  DefaultGravity.h \
  DefaultGround.h \
  DefaultPlanet.h \
  DiscBrake.h \
  DiscreteIntegrator.h \
  ExternalForceModel.h \
  FixedRootJoint.h \
  FixedRootJointFrame.h \
  Force.h \
  Gain.h \
  Gravity.h \
  Ground.h \
  Input.h \
  Integrator.h \
  Interact.h \
  Joint.h \
  Launchbar.h \
  LinearSpringDamper.h \
  MaxModel.h \
  MinModel.h \
  MobileRootJoint.h \
  MobileRootJointFrame.h \
  Output.h \
  Product.h \
  PrismaticJoint.h \
  PrismaticJointFrame.h \
  RevoluteActuator.h \
  RevoluteActuatorFrame.h \
  RevoluteJoint.h \
  RevoluteJointFrame.h \
  RigidBody.h \
  RootFrame.h \
  Saturation.h \
  Sensor.h \
  SimpleContact.h \
  SimpleGear.h \
  Summer.h \
  Table.h \
  Tailhook.h \
  Tank.h \
  TimeDerivative.h \
  TransferFunction.h \
  Turbulence.h \
  UnaryFunctionModel.h \
  Vehicle.h \
  WheelContact.h \
  Wind.h

OpenFDMModelsSOURCES = \
  AeroForce.cpp \
  AirSpring.cpp \
  Atmosphere.cpp \
  AtmosphereSTD1976.cpp \
  Bias.cpp \
  BinaryFunctionModel.cpp \
  ConstModel.cpp \
  Contact.cpp \
  DeadBand.cpp \
  DefaultGravity.cpp \
  DefaultGround.cpp \
  DefaultPlanet.cpp \
  DiscBrake.cpp \
  DiscreteIntegrator.cpp \
  ExternalForceModel.cpp \
  FixedRootJoint.cpp \
  Force.cpp \
  Gain.cpp \
  Gravity.cpp \
  Ground.cpp \
  Input.cpp \
  Integrator.cpp \
  Interact.cpp \
  Joint.cpp \
  Launchbar.cpp \
  LinearSpringDamper.cpp \
  MaxModel.cpp \
  MinModel.cpp \
  MobileRootJoint.cpp \
  Output.cpp \
  Product.cpp \
  PrismaticJoint.cpp \
  RevoluteActuator.cpp \
  RevoluteJoint.cpp \
  RigidBody.cpp \
  RootFrame.cpp \
  Saturation.cpp \
  SimpleContact.cpp \
  SimpleGear.cpp \
  Summer.cpp \
  Table.cpp \
  Tailhook.cpp \
  Tank.cpp \
  TimeDerivative.cpp \
  TransferFunction.cpp \
  UnaryFunctionModel.cpp \
  Vehicle.cpp \
  WheelContact.cpp

nobase_OpenFDMinclude_HEADERS = $(OpenFDMCoreHEADERS) $(OpenFDMModelsHEADERS)
libOpenFDM_la_SOURCES = $(OpenFDMCoreSOURCES) $(OpenFDMModelsSOURCES)
