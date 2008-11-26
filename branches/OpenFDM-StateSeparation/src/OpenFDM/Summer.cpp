/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Summer.h"

#include <string>
#include "Types.h"
#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Summer, SimpleDirectModel)
//   DEF_OPENFDM_PROPERTY(Unsigned, NumSummands, Serialized)
  END_OPENFDM_OBJECT_DEF

Summer::Summer(const std::string& name) :
  SimpleDirectModel(name)
{
}

Summer::~Summer(void)
{
}

void
Summer::output(Context& context) const
{
  if (!getNumInputPorts())
    return;
  context.getOutputValue() = context.getInputValue(0);
  for (unsigned i = 1; i < getNumInputPorts(); ++i)
    context.getOutputValue() += context.getInputValue(i);
}

// unsigned
// Summer::getNumSummands(void) const
// {
//   return getNumInputPorts();
// }

// void
// Summer::setNumSummands(unsigned num)
// {
//   unsigned oldnum = getNumSummands();
//   setNumInputPorts(num);
//   for (; oldnum < num; ++oldnum)
//     setInputPortName(oldnum, "+");
// }

// void
// Summer::setInputSign(unsigned num, Sign sign)
// {
//   if (getNumSummands() <= num)
//     return;
//   if (sign == Minus)
//     setInputPortName(num, "-");
//   else
//     setInputPortName(num, "+");
// }

// Summer::Sign
// Summer::getInputSign(unsigned num) const
// {
//   if (getNumSummands() <= num)
//     return Plus;
//   if (getInputPortName(num) == "-")
//     return Minus;
//   else
//     return Plus;
// }

} // namespace OpenFDM
