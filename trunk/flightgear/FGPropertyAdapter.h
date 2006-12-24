/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef FGPropertyAdapter_H
#define FGPropertyAdapter_H

#include <string>
#include <simgear/props/props.hxx>
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
  bool getPropertyValue(Variant& value) const
  {
    if (!mObject)
      return false;
    return mObject->getPropertyValue(mPropertyName, value);
  }
  bool setPropertyValue(const Variant& value)
  {
    if (!mObject)
      return false;

    return mObject->setPropertyValue(mPropertyName, value);
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
  {
    Variant variantValue;
    if (getPropertyValue(variantValue))
      mValue = variantValue.toString();
    return mValue.c_str();
  }
  
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
    Variant variantValue;
    if (getPropertyValue(variantValue))
      return false;
    Matrix m = variantValue.toMatrix();
    unsigned r = mIndex % rows(m);
    unsigned c = mIndex / rows(m);
    if (rows(m) <= r)
      return false;
    if (cols(m) <= c)
      return false;
    
    m(r, c) = value;
    return setPropertyValue(Variant(m));
  }
  /// Implements the SimGear property interface.
  virtual double getValue(void) const
  {
    if (mIndex == 1) {
      Variant variantValue;
      if (!getPropertyValue(variantValue))
        return 0;
      return variantValue.toReal();
    } else {
      Variant variantValue;
      if (!getPropertyValue(variantValue))
        return 0;
      Matrix m = variantValue.toMatrix();
      unsigned r = mIndex % rows(m);
      unsigned c = mIndex / rows(m);
      if (rows(m) <= r)
        return 0;
      if (cols(m) <= c)
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
  {
    Variant variantValue;
    if (!getPropertyValue(variantValue))
      return 0;
    return variantValue.toInteger();
  }

  virtual FGIntegerPropertyAdapter* clone(void) const
  { return new FGIntegerPropertyAdapter(*this); }
};

class FGInputCallback : public Input::Callback {
public:
  FGInputCallback(const SGPropertyNode* propertyNode) :
    mPropertyNode(propertyNode)
  { }
  virtual real_type getValue() const
  { return mPropertyNode->getDoubleValue(); }
private:
  SGSharedPtr<const SGPropertyNode> mPropertyNode;
};

class FGOutputCallback : public Output::Callback {
public:
  FGOutputCallback() : mValue(0)
  { }
  virtual void setValue(real_type value)
  { mValue = value; }
  real_type getValue(void) const
  { return mValue; }
private:
  real_type mValue;
};

class FGOutputReflector :
    public SGRawValue<double> {
public:
  FGOutputReflector(Output* output) :
    mOutputCallback(new FGOutputCallback)
  {
    if (!output)
      return;
    output->setCallback(mOutputCallback);
  }

  virtual bool setValue(double value)
  { return false; }
  
  virtual double getValue(void) const
  { return mOutputCallback->getValue(); }

  virtual FGOutputReflector* clone(void) const
  { return new FGOutputReflector(*this); }

private:
  SharedPtr<FGOutputCallback> mOutputCallback;
};

class FGRealPortReflector :
    public SGRawValue<double> {
public:
  FGRealPortReflector(NumericPortProvider* port, unsigned index = 1u) :
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
      NumericPortProvider* port = mPort;
      RealPortHandle realPortHandle = port->getPortInterface()->toRealPortInterface();
      if (realPortHandle.isConnected())
        return realPortHandle.getRealValue();
      else
        return 0;
    } else {
      NumericPortProvider* port = mPort;
      MatrixPortHandle matrixPortHandle = port->getPortInterface()->toMatrixPortInterface();
      if (matrixPortHandle.isConnected()) {
        Matrix m = matrixPortHandle.getMatrixValue();
        unsigned r = mIndex % rows(m);
        unsigned c = mIndex / rows(m);
        if (rows(m) <= r)
          return 0;
        if (cols(m) <= c)
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
  WeakPtr<NumericPortProvider> mPort;
};

} // namespace OpenFDM

#endif
