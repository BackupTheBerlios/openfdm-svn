/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <osg/Node>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/CullFace>
#include <osgDB/Archive>
#include <osgDB/ReadFile>

#include "Configuration.h"

namespace OpenFDM {

osg::Node*
genArrow(float d1, float d2, float l1, float l2, bool centered,
         const osg::Vec4& rgba, unsigned segs)
{
  osg::Group* arrow = new osg::Group();
  osg::Geode* arrowGeode = new osg::Geode();
  osg::Geometry* arrowGeometry = new osg::Geometry();
  
  arrowGeode->addDrawable(arrowGeometry);
  arrow->addChild(arrowGeode);
 
  osg::Vec3Array* arrowVertices = new osg::Vec3Array;
  osg::Vec3Array* arrowNormals = new osg::Vec3Array;
  float alpha = atan2f(d1, l1);
  float sina = sinf(alpha);
  float cosa = cosf(alpha);
  float xoff = centered ? 0 : 0.5*l2;
  for (unsigned i = 0; i < segs; ++i) {
    float sini = sinf(2*M_PI*i/segs);
    float cosi = cosf(2*M_PI*i/segs);
    float sini2 = sinf(2*M_PI*(i+0.5f)/segs);
    float cosi2 = cosf(2*M_PI*(i+0.5f)/segs);

    // The arrow tip.
    arrowVertices->push_back(osg::Vec3(0.5*l2+xoff, 0, 0));
    arrowNormals->push_back(osg::Vec3(sina, cosa*sini2, cosa*cosi2));

    arrowVertices->push_back(osg::Vec3(0.5f*l2-l1+xoff, d1*sini, d1*cosi));
    arrowNormals->push_back(osg::Vec3(sina, cosa*sini, cosa*cosi));


    // Arrow tip backside
    arrowVertices->push_back(osg::Vec3(0.5f*l2-l1+xoff, d1*sini, d1*cosi));
    arrowNormals->push_back(osg::Vec3(-1, 0, 0));

    arrowVertices->push_back(osg::Vec3(0.5f*l2-l1+xoff, d2*sini, d2*cosi));
    arrowNormals->push_back(osg::Vec3(-1, 0, 0));


    // Arrow body
    arrowVertices->push_back(osg::Vec3(0.5f*l2-l1+xoff, d2*sini, d2*cosi));
    arrowNormals->push_back(osg::Vec3(0, sini, cosi));

    arrowVertices->push_back(osg::Vec3(-0.5f*l2+xoff, d2*sini, d2*cosi));
    arrowNormals->push_back(osg::Vec3(0, sini, cosi));


    // Arrow backside surface
    arrowVertices->push_back(osg::Vec3(-0.5f*l2+xoff, d2*sini, d2*cosi));
    arrowNormals->push_back(osg::Vec3(-1, 0, 0));
  }
  arrowVertices->push_back(osg::Vec3(-0.5f*l2+xoff, 0, 0));
  arrowNormals->push_back(osg::Vec3(-1, 0, 0));

  arrowGeometry->setVertexArray(arrowVertices);
  arrowGeometry->setNormalArray(arrowNormals);
  arrowGeometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);


  osg::DrawElementsUInt* arrowTip =
    new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
  for (unsigned i = 0; i < segs; ++i) {
    arrowTip->push_back(7*i + 0);
    arrowTip->push_back(7*i + 1);
    arrowTip->push_back(7*((i+1)%segs) + 1);
  }
  arrowGeometry->addPrimitiveSet(arrowTip);
  
  osg::DrawElementsUInt* arrowTipBackside =
    new osg::DrawElementsUInt(osg::PrimitiveSet::QUAD_STRIP, 0);
  for (unsigned i = 0; i <= segs; ++i) {
    arrowTipBackside->push_back(7*(i%segs) + 2);
    arrowTipBackside->push_back(7*(i%segs) + 3);
  }
  arrowGeometry->addPrimitiveSet(arrowTipBackside);
  
  
  osg::DrawElementsUInt* arrowBody =
    new osg::DrawElementsUInt(osg::PrimitiveSet::QUAD_STRIP, 0);
  for (unsigned i = 0; i <= segs; ++i) {
    arrowBody->push_back(7*(i%segs) + 4);
    arrowBody->push_back(7*(i%segs) + 5);
  }
  arrowGeometry->addPrimitiveSet(arrowBody);

  
  osg::DrawElementsUInt* arrowBackside =
    new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_FAN, 0);
  arrowBackside->push_back(7*segs);
  for (unsigned i = segs; 0 <= i; --i)
    arrowBackside->push_back(7*(i%segs) + 6);
  arrowGeometry->addPrimitiveSet(arrowBackside);
  
  
  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->push_back(rgba);
  arrowGeometry->setColorArray(colors);
  arrowGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);

  osg::StateSet* stateSet = new osg::StateSet;
  osg::CullFace* cf = new osg::CullFace(osg::CullFace::FRONT);
  stateSet->setAttributeAndModes(cf);
  arrowGeometry->setStateSet(stateSet);

  return arrow;
}

osg::Node*
genCoordinateBullet(float diam)
{
  osgDB::ReaderWriter::Options* opts = new osgDB::ReaderWriter::Options;
  opts->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_ALL);
  osgDB::Registry::instance()->setOptions(opts);

  Configuration* config = Configuration::Instance();
  QByteArray bullet = config->getLibraryObject("bullet-selected.ac");
  return osgDB::readNodeFile(bullet.data());
}

} // namespace OpenFDM
