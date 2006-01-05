/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Referenced.h"
#include "Object.h"
#include "Variant.h"
#include "Property.h"

namespace OpenFDM {

UntypedPropertyImpl::~UntypedPropertyImpl(void)
{
}

Variant
UntypedPropertyImpl::getValue(void) /*const*/
{
  IntegerPropertyImpl* integerProperty = toIntegerPropertyImpl();
  if (integerProperty)
    return Variant(integerProperty->getValue());

  UnsignedPropertyImpl* unsignedProperty = toUnsignedPropertyImpl();
  if (unsignedProperty)
    return Variant(unsignedProperty->getValue());

  RealPropertyImpl* realProperty = toRealPropertyImpl();
  if (realProperty)
    return Variant(realProperty->getValue());

  Vector2PropertyImpl* vector2Property = toVector2PropertyImpl();
  if (vector2Property)
    return Variant(vector2Property->getValue());

  Vector3PropertyImpl* vector3Property = toVector3PropertyImpl();
  if (vector3Property)
    return Variant(vector3Property->getValue());

  QuaternionPropertyImpl* quaternionProperty = toQuaternionPropertyImpl();
  if (quaternionProperty)
    return Variant(quaternionProperty->getValue());

  PlanePropertyImpl* planeProperty = toPlanePropertyImpl();
  if (planeProperty) {
    Plane plane = planeProperty->getValue();
    Vector3 normal = plane.getNormal();
    return Variant(Vector4(normal(1), normal(2), normal(3), plane.getDist()));
  }

  Vector6PropertyImpl* vector6Property = toVector6PropertyImpl();
  if (vector6Property)
    return Variant(vector6Property->getValue());

  MatrixPropertyImpl* matrixProperty = toMatrixPropertyImpl();
  if (matrixProperty)
    return Variant(matrixProperty->getValue());

  StringPropertyImpl* stringProperty = toStringPropertyImpl();
  if (stringProperty)
    return Variant(stringProperty->getValue());

  return Variant();
}

void
UntypedPropertyImpl::setValue(const Variant& value)
{
  IntegerPropertyImpl* integerProperty = toIntegerPropertyImpl();
  if (integerProperty)
    integerProperty->setValue(value.toInteger());

  UnsignedPropertyImpl* unsignedProperty = toUnsignedPropertyImpl();
  if (unsignedProperty)
    unsignedProperty->setValue(value.toUnsigned());

  RealPropertyImpl* realProperty = toRealPropertyImpl();
  if (realProperty)
    realProperty->setValue(value.toReal());

  Vector2PropertyImpl* vector2Property = toVector2PropertyImpl();
  if (vector2Property) {
    Matrix m = value.toMatrix();
    if (m.rows() == 2 && m.cols() == 1)
      vector2Property->setValue(Vector2(m(1,1), m(1,2)));
  }

  Vector3PropertyImpl* vector3Property = toVector3PropertyImpl();
  if (vector3Property) {
    Matrix m = value.toMatrix();
    if (m.rows() == 3 && m.cols() == 1)
      vector3Property->setValue(Vector3(m(1,1), m(1,2), m(1,3)));
  }

  QuaternionPropertyImpl* quaternionProperty = toQuaternionPropertyImpl();
  if (quaternionProperty) {
    Matrix m = value.toMatrix();
    if (m.rows() == 4 && m.cols() == 1)
      quaternionProperty->setValue(Vector4(m(1,1), m(1,2), m(1,3), m(1,4)));
  }

  PlanePropertyImpl* planeProperty = toPlanePropertyImpl();
  if (planeProperty) {
    Matrix m = value.toMatrix();
    if (m.rows() == 4 && m.cols() == 1)
      planeProperty->setValue(Plane(Vector3(m(1,1), m(1,2), m(1,3)), m(1,4)));
  }

  Vector6PropertyImpl* vector6Property = toVector6PropertyImpl();
  if (vector6Property) {
    Matrix m = value.toMatrix();
    if (m.rows() == 6 && m.cols() == 1)
      vector6Property->setValue(Vector6(m(1,1), m(1,2), m(1,3),
                                        m(1,4), m(1,5), m(1,6)));
  }

  MatrixPropertyImpl* matrixProperty = toMatrixPropertyImpl();
  if (matrixProperty)
    matrixProperty->setValue(value.toMatrix());

  StringPropertyImpl* stringProperty = toStringPropertyImpl();
  if (stringProperty)
    stringProperty->setValue(value.toString());
}

IntegerPropertyImpl*
UntypedPropertyImpl::toIntegerPropertyImpl(void)
{
  return 0;
}

UnsignedPropertyImpl*
UntypedPropertyImpl::toUnsignedPropertyImpl(void)
{
  return 0;
}

RealPropertyImpl*
UntypedPropertyImpl::toRealPropertyImpl(void)
{
  return 0;
}

Vector2PropertyImpl*
UntypedPropertyImpl::toVector2PropertyImpl(void)
{
  return 0;
}

Vector3PropertyImpl*
UntypedPropertyImpl::toVector3PropertyImpl(void)
{
  return 0;
}

QuaternionPropertyImpl*
UntypedPropertyImpl::toQuaternionPropertyImpl(void)
{
  return 0;
}

PlanePropertyImpl*
UntypedPropertyImpl::toPlanePropertyImpl(void)
{
  return 0;
}

Vector6PropertyImpl*
UntypedPropertyImpl::toVector6PropertyImpl(void)
{
  return 0;
}

MatrixPropertyImpl*
UntypedPropertyImpl::toMatrixPropertyImpl(void)
{
  return 0;
}

StringPropertyImpl*
UntypedPropertyImpl::toStringPropertyImpl(void)
{
  return 0;
}

} // namespace OpenFDM
