/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Matrix_H
#define OpenFDM_Matrix_H

#include "Assert.h"
#include "Types.h"
#include "Math.h"
#include "LinAlg/Misc.h"
#include "LinAlg/Base.h"
#include "LinAlg/IO.h"
#include "LinAlg/Proxy.h"
#include "LinAlg/Array.h"
#include "LinAlg/Container.h"
#include "LinAlg/Expressions.h"
#include "LinAlg/Algorithm.h"

namespace OpenFDM {

using LinAlg::Zeros;
using LinAlg::Eye;
using LinAlg::Range;
using LinAlg::trans;
using LinAlg::Size;

typedef LinAlg::Matrix22<real_type> Matrix22;
typedef LinAlg::MatrixFactors<real_type,2,2,LinAlg::LUTag> Matrix22Factors;

typedef LinAlg::Matrix33<real_type> Matrix33;
typedef LinAlg::MatrixFactors<real_type,3,3,LinAlg::LUTag> Matrix33Factors;

typedef LinAlg::MatrixFactors<real_type,6,6,LinAlg::LUTag> Matrix66Factors;

typedef LinAlg::Matrix<real_type> Matrix;
typedef LinAlg::MatrixFactors<real_type,0,0,LinAlg::LUTag> MatrixFactors;

} // namespace OpenFDM

#endif
