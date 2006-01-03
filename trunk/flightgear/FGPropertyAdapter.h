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
    unsigned r = mIndex % rows(m);
    unsigned c = mIndex / rows(m);
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
      unsigned r = mIndex % rows(m);
      unsigned c = mIndex / rows(m);
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

} // namespace OpenFDM

#endif
