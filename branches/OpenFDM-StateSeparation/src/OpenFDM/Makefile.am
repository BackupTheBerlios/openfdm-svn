SUBDIRS = XML

INCLUDES = -I$(srcdir)/..

OpenFDMincludedir = $(includedir)/OpenFDM

lib_LTLIBRARIES = libOpenFDM.la
libOpenFDM_la_LIBADD = -lrt

OpenFDMCoreHEADERS = \
  AbstractNodeContext.h \
  AcceptorPortInfo.h \
  Assert.h \
  Atomic.h \
  BoolStateInfo.h \
  ConstFrameVisitor.h \
  ConstNodeVisitor.h \
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
  Group.h \
  ImplicitEuler.h \
  Inertia.h \
  Interval.h \
  Interact.h \
  LeafContext.h \
  LeafNode.h \
  LibraryModel.h \
  LibraryNode.h \
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
  MechanicBodyPort.h \
  MechanicInteractPort.h \
  MechanicNode.h \
  MechanicPortValue.h \
  MechanicAcceptorPortInfo.h \
  MechanicProviderPortInfo.h \
  Model.h \
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
  RigidBody.h \
  RootJoint.h \
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
  AcceptorPortInfo.cpp \
  Assert.cpp \
  Atomic.cpp \
  ConstNodeVisitor.cpp \
  ContinousStateValue.cpp \
  DoPri5.cpp \
  ExplicitAdams.cpp \
  ExplicitEuler.cpp \
  Frame.cpp \
  Function.cpp \
  Group.cpp \
  Interact.cpp \
  LeafNode.cpp \
  LibraryModel.cpp \
  LibraryNode.cpp \
  Logger.cpp \
  MatrixStateValue.cpp \
  MechanicNode.cpp \
  MechanicPortValue.cpp \
  MidpointRule.cpp \
  Model.cpp \
  Node.cpp \
  NodeVisitor.cpp \
  NumericAcceptorPortInfo.cpp \
  NumericPortValue.cpp \
  NumericProviderPortInfo.cpp \
  Object.cpp \
  ODESolver.cpp \
  Planet.cpp \
  PortInfo.cpp \
  PortValue.cpp \
  ProviderPortInfo.cpp \
  RigidBody.cpp \
  RootJoint.cpp \
  SampleTime.cpp \
  StateValue.cpp \
  Time.cpp \
  Unit.cpp \
  Variant.cpp

#   Environment.cpp \
#   EnvironmentObject.cpp \
#   ImplicitEuler.cpp \
#   Newton.cpp \
#   ReaderWriter.cpp \
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
  Delay.h \
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
  Delay.cpp \
  Gain.cpp \
  Integrator.cpp \
  Output.cpp

#   AeroForce.cpp \
#   AirSpring.cpp \
#   Atmosphere.cpp \
#   AtmosphereSTD1976.cpp \
#   Bias.cpp \
#   BinaryFunctionModel.cpp \
#   ConstModel.cpp \
#   Contact.cpp \
#   DeadBand.cpp \
#   DefaultGravity.cpp \
#   DefaultGround.cpp \
#   DefaultPlanet.cpp \
#   DiscBrake.cpp \
#   DiscreteIntegrator.cpp \
#   ExternalForceModel.cpp \
#   FixedRootJoint.cpp \
#   Force.cpp \
#   Gravity.cpp \
#   Ground.cpp \
#   Input.cpp \
#   Joint.cpp \
#   Launchbar.cpp \
#   LinearSpringDamper.cpp \
#   Mass.cpp \
#   MaxModel.cpp \
#   MinModel.cpp \
#   MobileRootJoint.cpp \
#   Output.cpp \
#   Product.cpp \
#   PrismaticJoint.cpp \
#   RevoluteActuator.cpp \
#   RevoluteJoint.cpp \
#   RigidBody.cpp \
#   RootFrame.cpp \
#   Saturation.cpp \
#   SimpleContact.cpp \
#   SimpleGear.cpp \
#   SimulationTime.cpp \
#   Summer.cpp \
#   Table.cpp \
#   Tailhook.cpp \
#   Tank.cpp \
#   TimeDerivative.cpp \
#   TransferFunction.cpp \
#   UnaryFunctionModel.cpp \
#   Vehicle.cpp \
#   WheelContact.cpp

nobase_OpenFDMinclude_HEADERS = $(OpenFDMCoreHEADERS) $(OpenFDMModelsHEADERS)
libOpenFDM_la_SOURCES = $(OpenFDMCoreSOURCES) $(OpenFDMModelsSOURCES)
