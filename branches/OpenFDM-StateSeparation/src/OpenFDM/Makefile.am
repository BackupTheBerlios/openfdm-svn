SUBDIRS = XML

INCLUDES = -I$(srcdir)/..

OpenFDMincludedir = $(includedir)/OpenFDM

lib_LTLIBRARIES = libOpenFDM.la
libOpenFDM_la_LIBADD = -lrt

if ENABLE_HDF5
libOpenFDM_la_LIBADD += -lhdf5
endif

OpenFDMCoreHEADERS = \
  AbstractModel.h \
  AbstractNodeContext.h \
  AbstractNodeInstance.h \
  AbstractSystem.h \
  Assert.h \
  Atomic.h \
  BoolStateInfo.h \
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
  Fraction.h \
  Frame.h \
  Function.h \
  Group.h \
  GroupInterfaceNode.h \
  GroupInput.h \
  GroupMechanicLink.h \
  GroupOutput.h \
  HDF5SystemOutput.h \
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
  MatrixStateInfo.h \
  MidpointRule.h \
  MechanicContext.h \
  MechanicLink.h \
  MechanicLinkValue.h \
  MechanicNode.h \
  Model.h \
  ModelContext.h \
  Mutex.h \
  MPFRFloat.h \
  Newton.h \
  Node.h \
  NodeInstance.h \
  NodeVisitor.h \
  NumericPortValue.h \
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
  SystemOutput.h \
  TableData.h \
  Task.h \
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
  AbstractModel.cpp \
  AbstractNodeContext.cpp \
  AbstractNodeInstance.cpp \
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
  GroupInterfaceNode.cpp \
  GroupInput.cpp \
  GroupMechanicLink.cpp \
  GroupOutput.cpp \
  Interact.cpp \
  LeafNode.cpp \
  LibraryModel.cpp \
  LibraryNode.cpp \
  Logger.cpp \
  MechanicContext.cpp \
  MechanicLinkValue.cpp \
  MechanicNode.cpp \
  MidpointRule.cpp \
  Model.cpp \
  ModelContext.cpp \
  Node.cpp \
  NodeInstance.cpp \
  NodeVisitor.cpp \
  NumericPortValue.cpp \
  Object.cpp \
  ODESolver.cpp \
  Planet.cpp \
  PortInfo.cpp \
  PortValue.cpp \
  RigidBody.cpp \
  RootJoint.cpp \
  StateValue.cpp \
  System.cpp \
  SystemOutput.cpp \
  Time.cpp \
  Unit.cpp \
  Variant.cpp

#   Environment.cpp \
#   EnvironmentObject.cpp \
#   ImplicitEuler.cpp \
#   Newton.cpp \
#   ReaderWriter.cpp \
#


OpenFDMModelsHEADERS = \
  AeroForce.h \
  AirSpring.h \
  Atmosphere.h \
  AtmosphereSTD1976.h \
  Bias.h \
  BinaryFunctionModel.h \
  CartesianJoint.h \
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
  Output.h \
  Product.h \
  PrismaticJoint.h \
  RevoluteActuator.h \
  RevoluteJoint.h \
  RotationalJoint.h \
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
  UnaryModel.h \
  UniversalJoint.h \
  Vehicle.h \
  WheelContact.h \
  Wind.h

OpenFDMModelsSOURCES = \
  Bias.cpp \
  ConstModel.cpp \
  DeadBand.cpp \
  Delay.cpp \
  DiscreteIntegrator.cpp \
  Gain.cpp \
  Input.cpp \
  Integrator.cpp \
  Output.cpp \
  Saturation.cpp \
  SimulationTime.cpp

OpenFDMMechanicSOURCES = \
  Joint.cpp \
  FixedRootJoint.cpp \
  Mass.cpp \
  MobileRootJoint.cpp \
  PrismaticJoint.cpp \
  RevoluteActuator.cpp \
  RevoluteJoint.cpp \
  RotationalJoint.cpp \
  Sensor.cpp \
  UnaryModel.cpp \
  UniversalJoint.cpp

#   AeroForce.cpp \
#   AirSpring.cpp \
#   Atmosphere.cpp \
#   AtmosphereSTD1976.cpp \
#   BinaryFunctionModel.cpp \
#   Contact.cpp \
#   DefaultGravity.cpp \
#   DefaultGround.cpp \
#   DefaultPlanet.cpp \
#   DiscBrake.cpp \
#   ExternalForceModel.cpp \
#   Force.cpp \
#   Gravity.cpp \
#   Ground.cpp \
#   Launchbar.cpp \
#   LinearSpringDamper.cpp \
#   MaxModel.cpp \
#   MinModel.cpp \
#   Output.cpp \
#   Product.cpp \
#   RigidBody.cpp \
#   SimpleContact.cpp \
#   SimpleGear.cpp \
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
libOpenFDM_la_SOURCES = $(OpenFDMCoreSOURCES) $(OpenFDMModelsSOURCES) $(OpenFDMMechanicSOURCES)
