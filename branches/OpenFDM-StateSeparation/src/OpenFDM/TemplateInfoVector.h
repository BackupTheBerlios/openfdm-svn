/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TemplateInfoVector_H
#define OpenFDM_TemplateInfoVector_H

#include <vector>
#include "SharedPtr.h"

namespace OpenFDM {

template<typename Info>
class TemplateInfoVector {
public:
  Info* getStateInfo(unsigned index)
  {
    if (mStateInfoVector.size() <= index)
      return 0;
    return mStateInfoVector[index];
  }
  const Info* getStateInfo(unsigned index) const // ???
  {
    if (mStateInfoVector.size() <= index)
      return 0;
    return mStateInfoVector[index];
  }
  unsigned size() const
  { return mStateInfoVector.size(); }
  void addStateInfo(Info* stateInfo)
  {
    if (!stateInfo)
      return;
    stateInfo->mIndex = mStateInfoVector.size();
    mStateInfoVector.push_back(stateInfo);
  }
  void removeStateInfo(Info* stateInfo)
  {
    if (!stateInfo)
      return;
    
    typename StateInfoVector::iterator i;
    i = std::find(mStateInfoVector.begin(), mStateInfoVector.end(), stateInfo);
    if (i == mStateInfoVector.end())
      return;
    unsigned index = (*i)->mIndex;
    i = mStateInfoVector.erase(i);
    for (;i != mStateInfoVector.end(); ++i, ++index)
      (*i)->mIndex = index;
  }
private:
  typedef std::vector<SharedPtr<Info> > StateInfoVector;
  StateInfoVector mStateInfoVector;
};


} // namespace OpenFDM

#endif
