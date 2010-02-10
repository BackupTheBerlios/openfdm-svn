/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Geometries_H
#define OpenFDM_Geometries_H

#include <osg/Vec4>
#include <osg/Node>

namespace OpenFDM {

extern osg::Node*
genArrow(float d1 = 0.02f, // The diameter of the arrow tip
         float d2 = 0.01f, // The arrow body diameter
         float l1 = 0.04f, // The length of the arrow tip
         float l2 = 1.0f,  // The arrow total length
         bool centered = true,  // If the arrow is centered or origin based
         const osg::Vec4& rgba = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), // colour
         unsigned segs = 32);   // The number of rotation segments.

osg::Node*
genCoordinateBullet(float diam = 1.0f);

} // namespace OpenFDM

#endif
