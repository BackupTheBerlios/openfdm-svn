/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef FGPropertyAdapter_H
#define FGPropertyAdapter_H

#include <string>
#include <simgear/props/props.hxx>
#include <OpenFDM/Property.h>
#include <OpenFDM/Model.h>

namespace OpenFDM {

class FGStringPropertyAdapter :
    public SGRawValue<const char*> {
public:
  FGStringPropertyAdapter(const StringProperty& stringProperty) :
    mStringProperty(stringProperty)
  {}
  virtual ~FGStringPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(const char* value)
  { return false; }
  /// Implements the SimGear property interface.
  virtual const char* getValue(void) const
  { mValue = mStringProperty.getValue(); return mValue.c_str(); }
  
  virtual FGStringPropertyAdapter* clone(void) const
  { return new FGStringPropertyAdapter(*this); }

private:
  mutable std::string mValue;
  mutable StringProperty mStringProperty; /*FIXME*/
};

class FGRealPropertyAdapter :
    public SGRawValue<double> {
public:
  FGRealPropertyAdapter(const RealProperty& realProperty) :
    mRealProperty(realProperty)
  { }
  virtual ~FGRealPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(double value)
  { mRealProperty.setValue(value); return mRealProperty.isValid(); }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  { return mRealProperty.getValue(); }
  
  virtual FGRealPropertyAdapter* clone(void) const
  { return new FGRealPropertyAdapter(*this); }

private:
  mutable RealProperty mRealProperty; /*FIXME*/
};

class FGIntPropertyAdapter :
    public SGRawValue<int> {
public:
  FGIntPropertyAdapter(const IntegerProperty& intProperty) :
    mIntProperty(intProperty)
  { }
  virtual ~FGIntPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(int value)
  { mIntProperty.setValue(value); return mIntProperty.isValid(); }
  /// Implements the SimGear property interface.
  virtual int getValue(void) const
  { return mIntProperty.getValue(); }
  
  virtual FGIntPropertyAdapter* clone(void) const
  { return new FGIntPropertyAdapter(*this); }

private:
  mutable IntegerProperty mIntProperty; /*FIXME*/
};

class FGUnsignedPropertyAdapter :
    public SGRawValue<int> {
public:
  FGUnsignedPropertyAdapter(const UnsignedProperty& unsignedProperty) :
    mUnsignedProperty(unsignedProperty)
  { }
  virtual ~FGUnsignedPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(int value)
  { if (value < 0) return false; mUnsignedProperty.setValue(value); return mUnsignedProperty.isValid(); }
  /// Implements the SimGear property interface.
  virtual int getValue(void) const
  { return mUnsignedProperty.getValue(); }
  
  virtual FGUnsignedPropertyAdapter* clone(void) const
  { return new FGUnsignedPropertyAdapter(*this); }

private:
  mutable UnsignedProperty mUnsignedProperty; /*FIXME*/
};

class FGVector2PropertyAdapter :
    public SGRawValue<double> {
public:
  FGVector2PropertyAdapter(const Vector2Property& property, unsigned idx) :
    mProperty(property), mIndex(idx)
  { if (mIndex < 1 || 2 < mIndex) mIndex = 1; }
  virtual ~FGVector2PropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(double value)
  { Vector2 tmp = mProperty.getValue(); tmp(mIndex) = value; mProperty.setValue(tmp); return mProperty.isValid(); }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  { Vector2 value = mProperty.getValue(); return value(mIndex); }
  
  virtual FGVector2PropertyAdapter* clone(void) const
  { return new FGVector2PropertyAdapter(*this); }

private:
  mutable Vector2Property mProperty; /*FIXME*/
  unsigned mIndex;
};

class FGVector3PropertyAdapter :
    public SGRawValue<double> {
public:
  FGVector3PropertyAdapter(const Vector3Property& property, unsigned idx) :
    mProperty(property), mIndex(idx)
  { if (mIndex < 1 || 3 < mIndex) mIndex = 1; }
  virtual ~FGVector3PropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(double value)
  { Vector3 tmp = mProperty.getValue(); tmp(mIndex) = value; mProperty.setValue(tmp); return mProperty.isValid(); }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  { Vector3 value = mProperty.getValue(); return value(mIndex); }
  
  virtual FGVector3PropertyAdapter* clone(void) const
  { return new FGVector3PropertyAdapter(*this); }

private:
  mutable Vector3Property mProperty; /*FIXME*/
  unsigned mIndex;
};

class FGQuaternionPropertyAdapter :
    public SGRawValue<double> {
public:
  FGQuaternionPropertyAdapter(const QuaternionProperty& property, unsigned idx) :
    mProperty(property), mIndex(idx)
  { if (mIndex < 1 || 4 < mIndex) mIndex = 1; }
  virtual ~FGQuaternionPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(double value)
  { Quaternion tmp = mProperty.getValue(); tmp(mIndex) = value; mProperty.setValue(tmp); return mProperty.isValid(); }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  { Quaternion value = mProperty.getValue(); return value(mIndex); }
  
  virtual FGQuaternionPropertyAdapter* clone(void) const
  { return new FGQuaternionPropertyAdapter(*this); }

private:
  mutable QuaternionProperty mProperty; /*FIXME*/
  unsigned mIndex;
};

class FGVector6PropertyAdapter :
    public SGRawValue<double> {
public:
  FGVector6PropertyAdapter(const Vector6Property& property, unsigned idx) :
    mProperty(property), mIndex(idx)
  { if (mIndex < 1 || 6 < mIndex) mIndex = 1; }
  virtual ~FGVector6PropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(double value)
  { Vector6 tmp = mProperty.getValue(); tmp(mIndex) = value; mProperty.setValue(tmp); return mProperty.isValid(); }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  { Vector6 value = mProperty.getValue(); return value(mIndex); }
  
  virtual FGVector6PropertyAdapter* clone(void) const
  { return new FGVector6PropertyAdapter(*this); }

private:
  mutable Vector6Property mProperty; /*FIXME*/
  unsigned mIndex;
};

class FGVariantPropertyAdapter :
    public SGRawValue<double> {
public:
  FGVariantPropertyAdapter(const Property& property) :
    mProperty(property)
  { }
  virtual ~FGVariantPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(double value)
  { mProperty.setValue(Variant(value)); return mProperty.isValid(); }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  { return mProperty.getValue().toReal(); }
  
  virtual FGVariantPropertyAdapter* clone(void) const
  { return new FGVariantPropertyAdapter(*this); }

private:
  mutable Property mProperty; /*FIXME*/
};

class FGRealInputModel :
    public Model {
public:
  FGRealInputModel(const std::string& name, SGPropertyNode* propertyNode) :
    Model(name),
    mPropertyNode(propertyNode)
  {
    setDirectFeedThrough(false);
  
//   setNumInputPorts(1);
//   setInputPortName(0, "input");
  
    setNumOutputPorts(1);
    setOutputPort(0, "value", Property(this, &FGRealInputModel::getValue));
  }
  virtual ~FGRealInputModel(void)
  {
  }

  void setPropertyNode(SGPropertyNode* propertyNode)
  { mPropertyNode = propertyNode; }
  SGPropertyNode* getPropertyNode(void)
  { return mPropertyNode.ptr(); }

  real_type getValue(void) const
  {
    if (mPropertyNode.valid())
      return mPropertyNode->getDoubleValue();
    else
      return 0;
  }
private:
  SGPropertyNode_ptr mPropertyNode;
};

class FGRealOutputModel :
    public Model {
public:
  FGRealOutputModel(const std::string& name, SGPropertyNode* propertyNode) :
    Model(name),
    mPropertyNode(propertyNode)
  {
    setDirectFeedThrough(false);
  
    setNumInputPorts(1);
    setInputPortName(0, "input");
  }
  virtual ~FGRealOutputModel(void)
  { }

  void setPropertyNode(SGPropertyNode* propertyNode)
  { mPropertyNode = propertyNode; }
  SGPropertyNode* getPropertyNode(void)
  { return mPropertyNode.ptr(); }

  virtual bool init(void)
  {
    OpenFDMAssert(getInputPort(0).isValid());
    return getInputPort(0).isValid();
  }

  virtual void output(void)
  {
    OpenFDMAssert(getInputPort(0).isValid());
    if (mPropertyNode.valid())
      mPropertyNode->setDoubleValue(getInputPort(0).getValue().toReal());
  }

private:
  SGPropertyNode_ptr mPropertyNode;
};

// This one is used to write changes to input properties into their input
// models
class InputChangeListener : public SGPropertyChangeListener {
public:
  InputChangeListener(Input* inputModel) : mInputModel(inputModel) {}
  virtual ~InputChangeListener(void) {}
  virtual void valueChanged(SGPropertyNode * node)
  {
    // Just to be sure
    if (!node)
      return;
    // Check if it is still valid
    if (!mInputModel)
      return;
    // Set the input from the nodes's value.
    mInputModel->setInputValue(node->getDoubleValue());
  }
private:
  // Holds the input model where it should write the value
  // Note that this shal not be a shared_ptr, since we get a recursive
  // ref count loop in that case.
  managed_ptr<Input> mInputModel;
};

// That class just takes care that the listeners to a specific Input are
// cleaned up past the input is deleted.
class InputChangeUserData : public Object {
public:
  InputChangeUserData(Input* inputModel, SGPropertyNode* node) :
    mListener(new InputChangeListener(inputModel))
  {
    node->addChangeListener(mListener);
    // Don't forget to set the initial value
    mListener->valueChanged(node);
  }
  virtual ~InputChangeUserData(void)
  {
    // Also deregisters itself at the SGPropertyNode.
    delete mListener;
  }
private:
  InputChangeListener* mListener;
};

} // namespace OpenFDM

#endif
