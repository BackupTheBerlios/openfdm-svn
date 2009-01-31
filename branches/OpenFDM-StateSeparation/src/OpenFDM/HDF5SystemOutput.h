/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_HDF5SystemOutput_H
#define OpenFDM_HDF5SystemOutput_H

#include <sstream>
#include <set>
#include <hdf5.h>
#include "Group.h"
#include "SystemOutput.h"
#include "ConstNodeVisitor.h"
#include "NodeInstance.h"

namespace OpenFDM {

class HDF5Object {
public:
  HDF5Object() : _id(-1) { }
  HDF5Object(hid_t id, bool newRef) : _id(-1)
  { if (newRef) assignNewRef(id); else assign(id); }
  HDF5Object(const HDF5Object& object) : _id(-1)
  { assign(object.getId()); }
  ~HDF5Object()
  { release(); }

  HDF5Object& operator=(const HDF5Object& object)
  { assign(object.getId()); return *this; }

  hid_t getId() const
  { return _id; }

  int getNumReferences() const
  { if (_id < 0) return 0; return H5Iget_ref(_id); }
  bool valid() const
  { return 0 <= _id && H5I_BADID != H5Iget_type(_id); }

  std::string getName() const
  {
    if (_id < 0)
      return std::string();
    ssize_t size = H5Iget_name(_id, 0, 0);
    char* tmp = new char[size];
    size = H5Iget_name(_id, tmp, size);
    std::string name(tmp, size);
    delete [] tmp;
    return name;
  }

  void release()
  {
    if (0 <= _id)
      H5Idec_ref(_id);
    _id = -1;
  }

protected:
  // Assign this object a new reference, the reference count is not incremented
  // in this method
  void assignNewRef(hid_t id)
  {
    release();
    _id = id;
  }
  void assign(hid_t id)
  {
    if (_id == id)
      return;
    if (0 <= id && H5I_BADID != H5Iget_type(id))
      H5Iinc_ref(id);
    release();
    _id = id;
  }
private:
  hid_t _id;
};

class HDF5Group : public HDF5Object {
public:
  HDF5Group()
  { }
  HDF5Group(const HDF5Object& object)
  {
    if (!object.valid())
      return;
    if (H5Iget_type(object.getId()) != H5I_GROUP)
      return;
    assign(object.getId());
  }
  HDF5Group(const HDF5Group& group)
  { assign(group.getId()); }
  HDF5Group(const HDF5Object& parent, const std::string& filename)
  { create(parent, filename); }

  HDF5Group& operator=(const HDF5Group& object)
  { assign(object.getId()); return *this; }

  bool create(const HDF5Object& parent, const std::string& name)
  {
    if (!parent.valid())
      return false;
    hid_t id;
#if (1 < H5_VERS_MAJOR || (1 == H5_VERS_MAJOR && 8 <= H5_VERS_MINOR))
    id = H5Gcreate(parent.getId(), name.c_str(), H5P_DEFAULT, H5P_DEFAULT,
                   H5P_DEFAULT);
#else
    id = H5Gcreate(parent.getId(), name.c_str(), 0);
#endif
    assignNewRef(id);
    return 0 <= id;
  }

  bool link(const HDF5Object& object, const std::string& name)
  {
    if (!valid())
      return false;
    if (!object.valid())
      return false;
    herr_t status = H5Glink2(object.getId(), ".", H5G_LINK_HARD,
                             getId(), name.c_str());
    return 0 <= status;
  }
};

class HDF5File : public HDF5Object {
public:
  HDF5File()
  { }
  HDF5File(const HDF5File& file)
  { assign(file.getId()); }
  HDF5File(const HDF5Object& object)
  {
    if (!object.valid())
      return;
    if (H5Iget_type(object.getId()) != H5I_FILE)
      return;
    assign(object.getId());
  }
  HDF5File(const std::string& filename)
  { open(filename); }
  HDF5File& operator=(const HDF5File& object)
  { assign(object.getId()); return *this; }
  bool open(const std::string& name)
  {
    hid_t id = H5Fcreate(name.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    assignNewRef(id);
    return 0 <= id;
  }
  bool flush()
  {
    return 0 == H5Fflush(getId(), H5F_SCOPE_LOCAL);
  }
};

class HDFMatrix : public HDF5Object {
public:
  HDFMatrix()
  { }
  HDFMatrix(const HDF5Object& parent, const std::string& name,
            const Matrix& matrix)
  { create(parent, name, matrix); }

  bool create(const HDF5Object& parent, const std::string& name,
              const Matrix& matrix)
  {
    if (!parent.valid())
      return false;

    hsize_t rank = 2;
    hsize_t dims[2] = { cols(matrix), rows(matrix) };

//     HDF5Object dataspace(H5Screate(H5S_NULL));
//     HDF5Object dataspace(H5Screate(H5S_SIMPLE));
    HDF5Object dataspace(H5Screate_simple(rank, dims, 0), true);
    if (!dataspace.valid())
      return false;

    hid_t id;
#if (1 < H5_VERS_MAJOR || (1 == H5_VERS_MAJOR && 8 <= H5_VERS_MINOR))
    id = H5Dcreate(parent.getId(), name.c_str(), H5T_NATIVE_DOUBLE,
                   dataspace.getId(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
#else
    id = H5Dcreate(parent.getId(), name.c_str(), H5T_NATIVE_DOUBLE,
                   dataspace.getId(), H5P_DEFAULT);
#endif
    if (id < 0)
      return false;
    assignNewRef(id);

    if (H5Dwrite(getId(), H5T_NATIVE_DOUBLE, H5S_ALL,
                 H5S_ALL, H5P_DEFAULT, matrix.find(0, 0)) < 0)
      return false;

    return true;
  }
};

class HDFMatrixStream : public HDF5Object {
public:
  HDFMatrixStream(const HDF5Object& parent, const std::string& name,
                  const Size& size)
  {
    hsize_t _chunklen(100);
    herr_t status;
    hsize_t rank = 3;
    _dims[0] = 1;
    if (size(0) == 1) {
      if (size(1) == 1) {
        rank = 1;
        _dims[1] = 0;
        _dims[2] = 0;
      } else {
        rank = 2;
        _dims[1] = size(1);
        _dims[2] = 0;
      }
    } else {
      if (size(1) == 1) {
        rank = 2;
        _dims[1] = size(0);
        _dims[2] = 0;
      } else {
        rank = 3;
        _dims[1] = size(1);
        _dims[2] = size(0);
      }
    }
    hsize_t maxdims[3] = { H5S_UNLIMITED, _dims[1], _dims[2] };
    _dataspace = HDF5Object(H5Screate_simple(rank, _dims, maxdims), true);
    if (!_dataspace.valid())
      return;

    _dims[0] = 0;

    hsize_t chunk_dims[3] = { _chunklen, _dims[1], _dims[2] };
    
    HDF5Object cparms(H5Pcreate(H5P_DATASET_CREATE), true);
    status = H5Pset_chunk(cparms.getId(), rank, chunk_dims);
    hid_t id;
#if (1 < H5_VERS_MAJOR || (1 == H5_VERS_MAJOR && 8 <= H5_VERS_MINOR))
    id = H5Dcreate(parent.getId(), name.c_str(), H5T_NATIVE_DOUBLE,
                   _dataspace.getId(), H5P_DEFAULT, cparms.getId(),
                   H5P_DEFAULT);
#else
    id = H5Dcreate(parent.getId(), name.c_str(), H5T_NATIVE_DOUBLE,
                   _dataspace.getId(), cparms.getId());
#endif
    assignNewRef(id);
  }

  void append(const real_type& scalar)
  {
    Matrix tmp(1, 1);
    tmp(0, 0) = scalar;
    append(tmp);
  }

  void append(const Matrix& matrix)
  {
    if (!valid())
      return;

    // increment size
    _dims[0] += 1;
    herr_t status = H5Dextend(getId(), _dims);

    HDF5Object filespace(H5Dget_space(getId()), true);
    hsize_t offset[3] = { _dims[0] - 1, 0, 0 };
    hsize_t dims1[3] = { 1, _dims[1], _dims[2] };
    status = H5Sselect_hyperslab(filespace.getId(), H5S_SELECT_SET,
                                 offset, NULL, dims1, NULL);

    status = H5Dwrite(getId(), H5T_NATIVE_DOUBLE,
                      _dataspace.getId(), filespace.getId(),
                      H5P_DEFAULT, matrix.find(0, 0));

    H5Sselect_none(filespace.getId());
  }

private:
  hsize_t _dims[3];
  HDF5Object _dataspace;
};

class HDF5SystemOutput : public SystemOutput {
public:
  HDF5SystemOutput(const std::string& filename, bool outputMechanics = true) :
    mHDF5File(filename + ".h5"),
    mToplevelGroup(mHDF5File, "System"),
    mTimeStream(mToplevelGroup, "t", Size(1, 1)),
    mOutputMechanics(outputMechanics)
  { }
  virtual ~HDF5SystemOutput()
  { }

  virtual void output(const real_type& t)
  {
    mTimeStream.append(t);
    DumperList::iterator i;
    for (i = mDumperList.begin(); i != mDumperList.end(); ++i)
      (*i)->append();
    mHDF5File.flush();
  }

  virtual void attachTo(const System* system)
  {
    mDumperList.clear();
    if (!system)
      return;
    Visitor visitor(mToplevelGroup, system, mOutputMechanics);
    system->getNode()->accept(visitor);
    mDumperList = visitor.mDumperList;
  }

private:
  HDF5File mHDF5File;
  HDF5Group mToplevelGroup;
  HDFMatrixStream mTimeStream;
  bool mOutputMechanics;

  struct Dumper : public Referenced {
    virtual ~Dumper() {}
    virtual void append() = 0;
    virtual HDF5Object getObject() = 0;
  };

  struct MatrixDumper : public Dumper {
    MatrixDumper(const NumericPortValue* numericPortValue,
                 const HDF5Object& parent, const std::string& name) :
      mNumericPortValue(numericPortValue),
      _stream(parent, name, size(mNumericPortValue->getValue()))
    { OpenFDMAssert(numericPortValue); }
    virtual void append()
    { _stream.append(mNumericPortValue->getValue()); }
    virtual HDF5Object getObject()
    { return _stream; }

    SharedPtr<const NumericPortValue> mNumericPortValue;
    HDFMatrixStream _stream;
  };

  struct MechanicDumper : public Dumper {
    MechanicDumper(const MechanicLinkValue* mechanicLinkValue,
                   const HDF5Object& parent, const std::string& name) :
      mMechanicLinkValue(mechanicLinkValue),
      _group(parent, name),
      _position(_group, "position", Size(3, 1)),
      _orientation(_group, "orientation", Size(4, 1)),
      _eulerAngle(_group, "eulerAngles", Size(3, 1)),
      _velocity(_group, "velocity", Size(6, 1)),
      _refVelocity(_group, "refVelocity", Size(6, 1)),
      _acceleration(_group, "acceleration", Size(6, 1)),
      _force(_group, "force", Size(6, 1)),
      _inertia(_group, "inertia", Size(6, 6))
    { }
    virtual void append()
    {
      const CoordinateSystem& cs = mMechanicLinkValue->getCoordinateSystem();
      _position.append(cs.getPosition());
      _orientation.append(cs.getOrientation());
      _eulerAngle.append(cs.getOrientation().getEuler());
      _velocity.append(mMechanicLinkValue->getSpVel());
      _refVelocity.append(mMechanicLinkValue->getReferenceVelocity());
      _acceleration.append(mMechanicLinkValue->getSpAccel());
      _force.append(mMechanicLinkValue->getForce());
      _inertia.append(mMechanicLinkValue->getInertia());
    }
    virtual HDF5Object getObject()
    { return _group; }

    SharedPtr<const MechanicLinkValue> mMechanicLinkValue;
    HDF5Group _group;
    HDFMatrixStream _position;
    HDFMatrixStream _orientation;
    HDFMatrixStream _eulerAngle;
    HDFMatrixStream _velocity;
    HDFMatrixStream _refVelocity;
    HDFMatrixStream _acceleration;
    HDFMatrixStream _force;
    HDFMatrixStream _inertia;
  };

  typedef std::list<SharedPtr<Dumper> > DumperList;
  DumperList mDumperList;

  class Visitor : public ConstNodeVisitor {
  public:
    Visitor(const HDF5Group& group, const System* system, bool outputMechanics) :
      mSystem(system),
      mCurrentGroup(group),
      mOutputMechanics(outputMechanics)
    { }
    
    SharedPtr<const System> mSystem;
    
    const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const
    {
      if (!mSystem)
        return 0;
      return mSystem->getNodeInstance(nodePath);
    }
    
    virtual void apply(const NumericPort& portInfo)
    {
      const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
      if (!nodeInstance)
        return;
      apply(portInfo, nodeInstance->getPortValue(portInfo));
    }
    virtual void apply(const MechanicLink& portInfo)
    {
      if (!mOutputMechanics)
        return;
      const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
      if (!nodeInstance)
        return;
      apply(portInfo, nodeInstance->getPortValue(portInfo));
    }
    
    virtual void apply(const NumericPort& portInfo,
                       const NumericPortValue* numericPortValue)
    {
      OpenFDMAssert(mCurrentPortValuesGroup.valid());
      std::string name = portInfo.getName();
      name = mCurrentPortValuesUniqueStringSet.makeUnique(name);
      
      if (mPortValueMap.find(numericPortValue) == mPortValueMap.end()) {
        MatrixDumper* dumper;
        dumper = new MatrixDumper(numericPortValue,
                                  mCurrentPortValuesGroup, name);
        mPortValueMap[numericPortValue] = dumper;
        mDumperList.push_back(dumper);
      } else {
        mCurrentPortValuesGroup.link(mPortValueMap.find(numericPortValue)->second->getObject(), name);
      }
    }
    virtual void apply(const MechanicLink& portInfo,
                       const MechanicLinkValue* mechanicLinkValue)
    {
      OpenFDMAssert(mCurrentPortValuesGroup.valid());
      std::string name = portInfo.getName();
      name = mCurrentPortValuesUniqueStringSet.makeUnique(name);
      
      if (mPortValueMap.find(mechanicLinkValue) == mPortValueMap.end()) {
        MechanicDumper* dumper;
        dumper = new MechanicDumper(mechanicLinkValue,
                                    mCurrentPortValuesGroup, name);
        mPortValueMap[mechanicLinkValue] = dumper;
        mDumperList.push_back(dumper);
      } else {
        mCurrentPortValuesGroup.link(mPortValueMap.find(mechanicLinkValue)->second->getObject(), name);
      }
    }
    
    void appendPortValues(const Node& node)
    {
      if (!node.getNumPorts())
        return;
      OpenFDMAssert(mCurrentGroup.valid());
      mCurrentPortValuesGroup = HDF5Group(mCurrentGroup, "portValues");
      node.traversePorts(*this);
      mCurrentPortValuesGroup = HDF5Group();
      mCurrentPortValuesUniqueStringSet = UniqueStringSet();
    }
    
    virtual void apply(const Node& node)
    {
      HDF5Group parentGroup = mCurrentGroup;
      OpenFDMAssert(mCurrentGroup.valid());
      std::string name = node.getName();
      name = mCurrentGroupUniqueStringSet.makeUnique(name);
      mCurrentGroup = HDF5Group(parentGroup, name);
      appendPortValues(node);
      mCurrentGroup = parentGroup;
    }
    virtual void apply(const Group& group)
    {
      HDF5Group parentGroup = mCurrentGroup;
      OpenFDMAssert(mCurrentGroup.valid());
      std::string name = group.getName();
      name = mCurrentGroupUniqueStringSet.makeUnique(name);
      mCurrentGroup = HDF5Group(parentGroup, name);
      
      appendPortValues(group);
      
      UniqueStringSet parentUniqueStringSet;
      parentUniqueStringSet.swap(mCurrentGroupUniqueStringSet);
      group.traverse(*this);
      parentUniqueStringSet.swap(mCurrentGroupUniqueStringSet);
      
      mCurrentGroup = parentGroup;
    }
    
    // Helper class that makes names unique ...
    struct UniqueStringSet {
      UniqueStringSet()
      { _strings.insert(""); }
      std::string makeUnique(const std::string& s)
      {
        if (_strings.find(s) == _strings.end()) {
          _strings.insert(s);
          return s;
        }
        std::string unique;
        unsigned id = 0;
        do {
          std::stringstream ss;
          ss << s << ++id;
          unique = ss.str();
        } while (_strings.find(unique) != _strings.end());
        return unique;
      }
      void swap(UniqueStringSet& other)
      { _strings.swap(other._strings); }
    private:
      std::set<std::string> _strings;
    };
    
    HDF5Group mCurrentGroup;

    bool mOutputMechanics;
    
    UniqueStringSet mCurrentPortValuesUniqueStringSet;
    UniqueStringSet mCurrentGroupUniqueStringSet;
    
    HDF5Group mCurrentPortValuesGroup;
    
    typedef std::map<const PortValue*,SharedPtr<Dumper> > PortValueMap;
    PortValueMap mPortValueMap;
    
    typedef std::list<SharedPtr<Dumper> > DumperList;
    DumperList mDumperList;
  };
};

} // namespace OpenFDM

#endif
