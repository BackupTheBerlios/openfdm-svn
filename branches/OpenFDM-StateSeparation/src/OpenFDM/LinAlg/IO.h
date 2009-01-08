/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Matrix_IO_H
#define OpenFDM_Matrix_IO_H

#include <iostream>
#include <iomanip>

namespace OpenFDM {

namespace LinAlg {

/** Write matrix to a stream.

    @param os Stream to write to.
    @param A Matrix to write.
*/
template<typename char_type, typename traits_type, typename Impl, size_type m, size_type n> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os, const MatrixRValue<Impl,m,n>& A)
{
  typedef Impl implementation_type;
  typedef typename implementation_type::value_type value_type;

  // Save old flags and set new ones
  std::ios::fmtflags old_flags
    = os.setf(std::ios::right, std::ios::adjustfield);
  os.setf(std::ios::scientific, std::ios::floatfield);

  // Get width or use a default of 9 if the width is unset
  int old_width = os.width();
  int width = old_width <= 0 ? 9 : old_width;
  // Get old precision.
  int old_prec = os.precision();

  size_type rows = A.asImpl().rows();
  size_type cols = A.asImpl().cols();
  os << os.widen('[');
  for (size_type i = 0; i < rows; ++i) {
    if (0 < i)
      os << os.widen(' ');

    for (size_type j = 0; j < cols; ++j) {
      value_type val = A.asImpl()(i, j);
    
      if (val == 0) {
        os << std::setw(width) << os.widen('0') << os.widen(' ');
      } else {
        int expo = static_cast<int>(floor(fabs(log10(fabs(val)))));
        
        if (expo < 100) {
          os << std::setprecision(width-7);
        } else {
          os << std::setprecision(width-8);
        }
        
        os << std::setw(width) << val << os.widen(' ');
      }
    }

    if (i < rows-1)
      os << os.widen('\n');
  }
  os << os.widen(']');

  // Restore old flags.
  os.flags(old_flags);
  os << std::setw(old_width) << std::setprecision(old_prec);

  return os;
}

} // namespace LinAlg

} // namespace OpenFDM

#endif
