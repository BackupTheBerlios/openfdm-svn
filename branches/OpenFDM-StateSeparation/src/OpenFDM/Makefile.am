SUBDIRS = XML

INCLUDES = -I$(srcdir)/..

OpenFDMincludedir = $(includedir)/OpenFDM

lib_LTLIBRARIES = libOpenFDM.la
libOpenFDM_la_LIBADD = -lrt

if ENABLE_HDF5
libOpenFDM_la_LIBADD += -lhdf5
endif

OpenFDMCoreHEADERS = \
  AbstractAtmosphere.h \
  AbstractModel.h \
  AbstractNodeContext.h \
  AbstractNodeInstance.h \
  AbstractPlanet.h \
  AbstractSystem.h \
  AbstractWind.h \
  Assert.h \
  Atomic.h \
  BoolStateInfo.h \
  Connect.h \
  ConstNodeVisitor.h \
  ContinousStateInfo.h \
  ContinousStateInfoVector.h \
  ContinousStateValue.h \
  ContinousStateValueVector.h \
  CoordinateSystem.h \
  CowPtr.h \
  DiscreteStateValueVector.h \
  DoubleLinkInteract.h \
  DoPri5.h \
  Environment.h \
  ExplicitAdams.h \
  ExplicitEuler.h \
  FlatPlanet.h \
  Fraction.h \
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
  Port.h \
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
  SingleLinkInteract.h \
  SphericalEarth.h \
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
  WGS84Planet.h \
  XMLDumpModelVisitor.h

OpenFDMCoreSOURCES = \
  AbstractAtmosphere.cpp \
  AbstractGround.cpp \
  AbstractModel.cpp \
  AbstractNodeContext.cpp \
  AbstractNodeInstance.cpp \
  AbstractPlanet.cpp \
  AbstractWind.cpp \
  Assert.cpp \
  AtmosphereSTD1976.cpp \
  Atomic.cpp \
  Connect.cpp \
  ConstNodeVisitor.cpp \
  ContinousStateValue.cpp \
  DoPri5.cpp \
  Environment.cpp \
  ExplicitAdams.cpp \
  ExplicitEuler.cpp \
  FlatPlanet.cpp \
  Function.cpp \
  Group.cpp \
  GroupInterfaceNode.cpp \
  GroupInput.cpp \
  GroupMechanicLink.cpp \
  GroupOutput.cpp \
  ImplicitEuler.cpp \
  Interact.cpp \
  LeafNode.cpp \
  LibraryModel.cpp \
  LibraryNode.cpp \
  Logger.cpp \
  MechanicContext.cpp \
  MechanicLink.cpp \
  MechanicLinkValue.cpp \
  MechanicNode.cpp \
  MidpointRule.cpp \
  Model.cpp \
  ModelContext.cpp \
  Newton.cpp \
  Node.cpp \
  NodeInstance.cpp \
  NodeVisitor.cpp \
  NumericPortValue.cpp \
  Object.cpp \
  ODESolver.cpp \
  Port.cpp \
  PortValue.cpp \
  RigidBody.cpp \
  RootJoint.cpp \
  SphericalEarth.cpp \
  StateValue.cpp \
  System.cpp \
  SystemOutput.cpp \
  Time.cpp \
  Unit.cpp \
  Variant.cpp \
  WGS84Planet.cpp

#   ReaderWriter.cpp



OpenFDMModelsHEADERS = \
  Actuator.h \
  AirSpring.h \
  AtmosphereSTD1976.h \
  Bias.h \
  BinaryFunction.h \
  BreakPointLookup.h \
  ConstModel.h \
  DeadBand.h \
  Delay.h \
  DiscBrake.h \
  DiscreteIntegrator.h \
  Gain.h \
  Input.h \
  Integrator.h \
  LinearSpringDamper.h \
  MatrixConcat.h \
  MatrixSplit.h \
  MaxModel.h \
  MinModel.h \
  Output.h \
  Product.h \
  Saturation.h \
  SimpleDirectModel.h \
  SimulationTime.h \
  Summer.h \
  Table.h \
  TimeDerivative.h \
  TransferFunction.h \
  UnaryFunction.h \
  UnitConversion.h

OpenFDMModelsSOURCES = \
  Actuator.cpp \
  AirSpring.cpp \
  Bias.cpp \
  BinaryFunction.cpp \
  BreakPointLookup.cpp \
  ConstModel.cpp \
  DeadBand.cpp \
  Delay.cpp \
  DiscBrake.cpp \
  DiscreteIntegrator.cpp \
  Gain.cpp \
  Input.cpp \
  Integrator.cpp \
  LinearSpringDamper.cpp \
  MatrixConcat.cpp \
  MatrixSplit.cpp \
  MaxModel.cpp \
  MinModel.cpp \
  Output.cpp \
  Product.cpp \
  Saturation.cpp \
  SimpleDirectModel.cpp \
  SimulationTime.cpp \
  Summer.cpp \
  Table.cpp \
  TimeDerivative.cpp \
  TransferFunction.cpp \
  UnaryFunction.cpp \
  UnitConversion.cpp

OpenFDMMechanicHEADERS = \
  AeroForce.h \
  CartesianJoint.h \
  Contact.h \
  ExternalInteract.h \
  FixedRootJoint.h \
  InternalInteract.h \
  Joint.h \
  Mass.h \
  MobileRootJoint.h \
  Pacejka89.h \
  Pacejka94.h \
  PacejkaSimple.h \
  PacejkaTire.h \
  PrismaticActuator.h \
  PrismaticJoint.h \
  RevoluteActuator.h \
  RevoluteJoint.h \
  RotationalJoint.h \
  SimpleContact.h \
  SimpleGear.h \
  Tank.h \
  UniversalJoint.h \
  WheelContact.h

OpenFDMMechanicSOURCES = \
  Contact.cpp \
  DoubleLinkInteract.cpp \
  ExternalInteract.cpp \
  FixedRootJoint.cpp \
  InternalInteract.cpp \
  Joint.cpp \
  Mass.cpp \
  MobileRootJoint.cpp \
  Pacejka89.cpp \
  Pacejka94.cpp \
  PacejkaSimple.cpp \
  PacejkaTire.cpp \
  PrismaticActuator.cpp \
  PrismaticJoint.cpp \
  RevoluteActuator.cpp \
  RevoluteJoint.cpp \
  RotationalJoint.cpp \
  SimpleContact.cpp \
  SimpleGear.cpp \
  SingleLinkInteract.cpp \
  UniversalJoint.cpp \
  WheelContact.cpp


#   AeroForce.cpp \
#   Tank.cpp

#   Launchbar.cpp \
#   Tailhook.cpp \
#

nobase_OpenFDMinclude_HEADERS = \
  $(OpenFDMCoreHEADERS) \
  $(OpenFDMModelsHEADERS) \
  $(OpenFDMMechanicHEADERS)
libOpenFDM_la_SOURCES = \
  $(OpenFDMCoreSOURCES) \
  $(OpenFDMModelsSOURCES) \
  $(OpenFDMMechanicSOURCES)
