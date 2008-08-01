SUBDIRS = XML

INCLUDES = -I$(srcdir)/..

OpenFDMincludedir = $(includedir)/OpenFDM

lib_LTLIBRARIES = libOpenFDM.la
libOpenFDM_la_LIBADD = -lrt

OpenFDMCoreHEADERS = \
  AcceptorPortInfo.h \
  Assert.h \
  Atomic.h \
  BoolStateInfo.h \
  ConstFrameVisitor.h \
  ContinousStateInfo.h \
  ContinousStateInfoVector.h \
  ContinousStateValue.h \
  ContinousStateValueVector.h \
  CowPtr.h \
  DiscreteStateValueVector.h \
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
  Math.h \
  Matrix.h \
  MatrixInputPort.h \
  MatrixOutputPort.h \
  MatrixStateValue.h \
  MatrixStateInfo.h \
  MidpointRule.h \
  Model.h \
  ModelGroup.h \
  ModelVisitor.h \
  Mutex.h \
  MPFRFloat.h \
  Newton.h \
  Node.h \
  NodeVisitor.h \
  NumericPortValue.h \
  NumericAcceptorPortInfo.h \
  NumericProviderPortInfo.h \
  Object.h \
  ODESolver.h \
  OpenFDMConfig.h \
  PhysicalDimension.h \
  Plane.h \
  Planet.h \
  PortId.h \
  PortInfo.h \
  PortValue.h \
  PortValueList.h \
  ProviderPortInfo.h \
  Quaternion.h \
  ReaderWriter.h \
  RealInputPort.h \
  RealOutputPort.h \
  RealStateInfo.h \
  Referenced.h \
  Rotation.h \
  SampleTime.h \
  ScopeLock.h \
  SharedPtr.h \
  StateStream.h \
  StateInfo.h \
  StateInfoVector.h \
  StateValue.h \
  System.h \
  TableData.h \
  TaskInfo.h \
  TemplateContinousStateInfo.h \
  TemplateDiscreteStateInfo.h \
  TemplateInfoVector.h \
  TemplateStateInfo.h \
  TemplateValueVector.h \
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
  ContinousStateValue.cpp \
  Frame.cpp \
  Logger.cpp \
  MatrixStateValue.cpp \
  Node.cpp \
  NodeVisitor.cpp \
  NumericPortValue.cpp \
  Object.cpp \
  Planet.cpp \
  PortInfo.cpp \
  PortValue.cpp \
  SampleTime.cpp \
  StateValue.cpp \
  Time.cpp \
  Unit.cpp \
  Variant.cpp

#   DoPri5.cpp \
#   Environment.cpp \
#   EnvironmentObject.cpp \
#   ExplicitAdams.cpp \
#   ExplicitEuler.cpp \
#   Function.cpp \
#   GroupInput.cpp \
#   GroupOutput.cpp \
#   ImplicitEuler.cpp \
#   Newton.cpp \
#   MidpointRule.cpp \
#   Model.cpp \
#   ModelGroup.cpp \
#   ReaderWriter.cpp \
#   ODESolver.cpp \
#   System.cpp \
#   TaskInfo.cpp \
#


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
  Mass.h \
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
  SimulationTime.h \
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
  Mass.cpp \
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
  SimulationTime.cpp \
  Summer.cpp \
  Table.cpp \
  Tailhook.cpp \
  Tank.cpp \
  TimeDerivative.cpp \
  TransferFunction.cpp \
  UnaryFunctionModel.cpp \
  Vehicle.cpp \
  WheelContact.cpp

# nobase_OpenFDMinclude_HEADERS = $(OpenFDMCoreHEADERS) $(OpenFDMModelsHEADERS)
nobase_OpenFDMinclude_HEADERS = $(OpenFDMCoreHEADERS)
# libOpenFDM_la_SOURCES = $(OpenFDMCoreSOURCES) $(OpenFDMModelsSOURCES)
libOpenFDM_la_SOURCES = $(OpenFDMCoreSOURCES)
