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

template<typename T>
class FGPropertyAdapter :
    public SGRawValue<T> {
public:
  FGPropertyAdapter(Object* object, const std::string& propertyName) :
    mObject(object), mPropertyName(propertyName)
  {}

protected:
  Variant getPropertyValue(void) const
  {
    if (!mObject)
      return Variant();

    return mObject->getPropertyValue(mPropertyName);
  }
  bool setPropertyValue(const Variant& value)
  {
    if (!mObject)
      return false;

    // FIXME: check if settable ...
    mObject->setPropertyValue(mPropertyName, value);
    return true;
  }

private:
  std::string mPropertyName;
  WeakPtr<Object> mObject;
};


class FGStringPropertyAdapter :
    public FGPropertyAdapter<const char*> {
public:
  FGStringPropertyAdapter(Object* object, const std::string& propertyName) :
    FGPropertyAdapter<const char*>(object, propertyName)
  {}
  virtual ~FGStringPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(const char* value)
  { return setPropertyValue(Variant(std::string(value))); }
  /// Implements the SimGear property interface.
  virtual const char* getValue(void) const
  { mValue = getPropertyValue().toString(); return mValue.c_str(); }
  
  virtual FGStringPropertyAdapter* clone(void) const
  { return new FGStringPropertyAdapter(*this); }

private:
  mutable std::string mValue;
};


class FGRealPropertyAdapter :
    public FGPropertyAdapter<double> {
public:
  FGRealPropertyAdapter(Object* object, const std::string& propertyName,
                        unsigned index = 1) :
    FGPropertyAdapter<double>(object, propertyName), mIndex(index)
  {}
  virtual ~FGRealPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(double value)
  {
    Matrix m = getPropertyValue().toMatrix();
    unsigned r = mIndex % rows(m) + 1;
    unsigned c = mIndex / rows(m) + 1;
    if (r < 1 || rows(m) < r)
      return false;
    if (c < 1 || cols(m) < c)
      return false;
    
    m(r, c) = value;
    return setPropertyValue(Variant(m));
  }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  {
    if (mIndex == 1)
      return getPropertyValue().toReal();
    else {
      Matrix m = getPropertyValue().toMatrix();
      unsigned r = mIndex % rows(m) + 1;
      unsigned c = mIndex / rows(m) + 1;
      if (r < 1 || rows(m) < r)
        return 0;
      if (c < 1 || cols(m) < c)
        return 0;
      return m(r, c);
    }
  }
  
  virtual FGRealPropertyAdapter* clone(void) const
  { return new FGRealPropertyAdapter(*this); }
private:
  unsigned mIndex;
};

class FGIntegerPropertyAdapter :
    public FGPropertyAdapter<int> {
public:
  FGIntegerPropertyAdapter(Object* object, const std::string& propertyName) :
    FGPropertyAdapter<int>(object, propertyName)
  {}
  virtual ~FGIntegerPropertyAdapter(void) {}

  /// Implements the SimGear property interface.
  virtual bool setValue(int value)
  { return setPropertyValue(Variant(value)); }
  /// Implements the SimGear property interface.
  virtual int getValue(void) const
  { return getPropertyValue().toInteger(); }

  virtual FGIntegerPropertyAdapter* clone(void) const
  { return new FGIntegerPropertyAdapter(*this); }
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
  // Note that this shal not be a SharedPtr, since we get a recursive
  // ref count loop in that case.
  WeakPtr<Input> mInputModel;
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
    // is deleted in the property system, don't do here
    // FIXME: this might be a place where we can use the new refcounting thing
    // of flightgear
//     delete mListener;
  }
private:
  InputChangeListener* mListener;
};

class FGOutputReflector :
    public SGRawValue<double> {
public:
  FGOutputReflector(Output* output) :
    mOutputModel(output)
  {}

  virtual bool setValue(double value)
  { return false; }
  
  virtual double getValue(void) const
  {
    if (!mOutputModel)
      return 0;
    return mOutputModel->getValue();
  }

  virtual FGOutputReflector* clone(void) const
  { return new FGOutputReflector(*this); }

private:
  // Holds the output model where it should write the value
  // Note that this shal not be a SharedPtr, since we get a recursive
  // ref count loop in that case.
  WeakPtr<Output> mOutputModel;
};

class FGRealPortReflector :
    public SGRawValue<double> {
public:
  FGRealPortReflector(Port* port, unsigned index = 1u) :
    mPort(port), mIndex(index)
  {}

  /// Implements the SimGear property interface.
  virtual bool setValue(double value)
  { return false; }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  {
    if (!mPort)
      return 0;
    if (mIndex == 1) {
      const Port* port = mPort;
      RealPortHandle realPortHandle = const_cast<Port*>(port)->toRealPortHandle();
      if (realPortHandle.isConnected())
        return realPortHandle.getRealValue();
      else
        return 0;
    } else {
      const Port* port = mPort;
      MatrixPortHandle matrixPortHandle = const_cast<Port*>(port)->toMatrixPortHandle();
      if (matrixPortHandle.isConnected()) {
        Matrix m = matrixPortHandle.getMatrixValue();
        unsigned r = mIndex % rows(m) + 1;
        unsigned c = mIndex / rows(m) + 1;
        if (r < 1 || rows(m) < r)
          return 0;
        if (c < 1 || cols(m) < c)
          return 0;
        return m(r, c);
      } else
        return 0;
    }
  }
  
  virtual FGRealPortReflector* clone(void) const
  { return new FGRealPortReflector(*this); }

private:
  unsigned mIndex;
  WeakPtr<Port> mPort;
};

} // namespace OpenFDM

#endif
