/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_HDF5Writer_H
#define OpenFDM_HDF5Writer_H

#include <sstream>
#include <set>
#include <map>
#include <hdf5.h>
#include <OpenFDM/System.h>
#include <OpenFDM/ConstNodeVisitor.h>
#include <OpenFDM/NodeInstance.h>

namespace OpenFDM {

class HDF5Object : public Referenced {
public:
  HDF5Object() : _id(-1) { }
  HDF5Object(hid_t id) : _id(-1) { assign(id); }
  HDF5Object(const HDF5Object& object) : _id(-1)
  { assign(object.getId()); }
  virtual ~HDF5Object()
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
    if (_id == id)
      return;
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
  HDF5Group(const HDF5Object& parent, const std::string& filename)
  { create(parent, filename); }

  bool create(const HDF5Object& parent, const std::string& name)
  {
    hid_t id;
#if (1 < H5_VERS_MAJOR || (1 == H5_VERS_MAJOR && 8 <= H5_VERS_MINOR))
    id = H5Gcreate(parent.getId(), name.c_str(), H5P_DEFAULT, H5P_DEFAULT,
                   H5P_DEFAULT);
#else
    id = H5Gcreate(parent.getId(), name.c_str(), 0);
#endif
    assignNewRef(id);
    return 0 < id;
  }

//   bool link(const HDF5Object& object, const std::string& name)
//   {
//     if (!valid())
//       return false;
//     if (!object.valid())
//       return false;
//     int status = H5Olink(object.getId(), getId(), name.c_str(),
//                          H5P_DEFAULT, H5P_DEFAULT);
//     return 0 <= status;
//   }
};

class HDF5File : public HDF5Object {
public:
  HDF5File()
  { }
  HDF5File(const std::string& filename)
  { open(filename); }
  void open(const std::string& name)
  {
    hid_t id = H5Fcreate(name.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    assignNewRef(id);
  }
};

class HDFMatrixStream : public HDF5Object {
public:
  HDFMatrixStream(const HDF5Object& parent, const std::string& name) :
    _name(name),
    _chunklen(100),
    _parent(parent)
  {
  }

  void append(const real_type& scalar)
  {
    Matrix tmp(1, 1);
    tmp(0, 0) = scalar;
    append(tmp);
  }

  void append(const Matrix& matrix)
  {
    herr_t status;
    if (!valid()) {
      hsize_t rank = 3;
      if (cols(matrix) == 1) {
        rank = 2;
        if (rows(matrix) == 1)
          rank = 1;
      }
      _dims[0] = 1;
      _dims[1] = rows(matrix);
      _dims[2] = cols(matrix);
      hsize_t maxdims[3] = { H5S_UNLIMITED, _dims[1], _dims[2] };
      _dataspace = H5Screate_simple(rank, _dims, maxdims);
      if (!_dataspace.valid())
        return;

      hsize_t chunk_dims[3] = { _chunklen, rows(matrix), cols(matrix) };

      hid_t cparms = H5Pcreate(H5P_DATASET_CREATE);
      status = H5Pset_chunk(cparms, rank, chunk_dims);
      hid_t id;
#if (1 < H5_VERS_MAJOR || (1 == H5_VERS_MAJOR && 8 <= H5_VERS_MINOR))
      id = H5Dcreate(_parent.getId(), _name.c_str(), H5T_NATIVE_DOUBLE,
                      _dataspace.getId(), H5P_DEFAULT, cparms, H5P_DEFAULT);
#else
      id = H5Dcreate(_parent.getId(), _name.c_str(), H5T_NATIVE_DOUBLE,
                      _dataspace.getId(), cparms);
#endif
      assignNewRef(id);
      H5Pclose(cparms);
    } else {
      // increment size
      _dims[0] += 1;
      status = H5Dextend(getId(), _dims);
    }

    HDF5Object filespace = H5Dget_space(getId());
    hsize_t offset[3] = { _dims[0] - 1, 0, 0 };
    hsize_t dims1[3] = { 1, _dims[1], _dims[2] };
    status = H5Sselect_hyperslab(filespace.getId(), H5S_SELECT_SET,
                                 offset, NULL, dims1, NULL);

    std::vector<double> data(_dims[1]*_dims[2]);
    for (hsize_t i = 0; i < _dims[1]; ++i)
      for (hsize_t j = 0; j < _dims[2]; ++j)
        // FIXME?? row or column major ...
        data[j + i*_dims[2]] = matrix(i, j);
        // data[i + j*_dims[1]] = matrix(i, j);
    status = H5Dwrite(getId(), H5T_NATIVE_DOUBLE,
                      _dataspace.getId(), filespace.getId(),
                      H5P_DEFAULT, &data.front());
  }

private:
  std::string _name;
  hsize_t _dims[3];
  HDF5Object _dataspace;

  hsize_t _chunklen;

  HDF5Object _parent;
};

class DataLogObject : public ConstNodeVisitor {
public:
  virtual ~DataLogObject() {}

  virtual void output(const real_type& t) = 0;

  void attachTo(const System* system)
  {
    mNodeInstanceMap.clear();
    if (!system)
      return;
    // Build an index to the system nodes
    ConstNodeInstanceList::const_iterator i;
    for (i = system->getNodeInstanceList().begin();
         i != system->getNodeInstanceList().end(); ++i) {
      mNodeInstanceMap[(*i)->getNodePath()] = *i;
    }
    system->getNode()->accept(*this);
  }

  virtual void apply(const PortInfo* portInfo, const PortValue* portValue)
  { }
  virtual void apply(const PortInfo* portInfo,
                     const NumericPortValue* numericPortValue)
  { apply(portInfo, static_cast<const PortValue*>(numericPortValue)); }
  virtual void apply(const PortInfo* portInfo,
                     const MechanicPortValue* mechanicPortValue)
  { apply(portInfo, static_cast<const PortValue*>(mechanicPortValue)); }

protected:
  const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const
  {
    NodeInstanceMap::const_iterator i = mNodeInstanceMap.find(nodePath);
    if (i == mNodeInstanceMap.end())
      return 0;
    return i->second;
  }
  void appendPortValues(const Node&)
  {
    const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
    appendPortValues(*nodeInstance);
  }
  void appendPortValues(const AbstractNodeInstance& nodeInstance)
  {
    unsigned numPorts = nodeInstance.getNode().getNumPorts();
    for (unsigned i = 0; i < numPorts; ++i) {
      const PortValue* portValue;
      portValue = nodeInstance.getPortValueList().getPortValue(i);
      const NumericPortValue* npv;
      npv = dynamic_cast<const NumericPortValue*>(portValue);
      if (npv) {
        apply(nodeInstance.getNode().getPort(i), npv);
        continue;
      }

      const MechanicPortValue* mpv;
      mpv = dynamic_cast<const MechanicPortValue*>(portValue);
      if (npv) {
        apply(nodeInstance.getNode().getPort(i), mpv);
        continue;
      }

      apply(nodeInstance.getNode().getPort(i), portValue);
    }
  }

private:
  typedef std::map<NodePath, SharedPtr<const AbstractNodeInstance> > NodeInstanceMap;
  NodeInstanceMap mNodeInstanceMap;
};


class HDF5Log : public DataLogObject {
public:
  HDF5Log(const std::string& filename) :
    mHDF5File(filename),
    mCurrentGroup(mHDF5File, "System"),
    mTimeStream(mCurrentGroup.getId(), "t")
  { }
  ~HDF5Log()
  { }

// private:
  void output(const real_type& t)
  {
    mTimeStream.append(t);
    DumperList::iterator i;
    for (i = mDumperList.begin(); i != mDumperList.end(); ++i)
      (*i)->append();
  }

  virtual void apply(const PortInfo* portInfo,
                     const NumericPortValue* numericPortValue)
  {
    OpenFDMAssert(mCurrentPortValuesGroup.valid());
    std::string name = portInfo->getName();
    name = mCurrentPortValuesUniqueStringSet.makeUnique(name);
    mDumperList.push_back(new MatrixDumper(numericPortValue, mCurrentPortValuesGroup, name));
  }

  void appendPortValues(const Node& node)
  {
    OpenFDMAssert(mCurrentGroup.valid());
    mCurrentPortValuesGroup = HDF5Group(mCurrentGroup, "portValues");
    DataLogObject::appendPortValues(node);
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

  HDF5File mHDF5File;
  HDF5Group mCurrentGroup;
  HDFMatrixStream mTimeStream;

  UniqueStringSet mCurrentPortValuesUniqueStringSet;
  UniqueStringSet mCurrentGroupUniqueStringSet;

  HDF5Group mCurrentPortValuesGroup;

  // Only hdf5 version >= 1.8 can do hard links
//   typedef std::map<const PortValue*, SharedPtr<HDF5Object> > PortValueMap;
//   PortValueMap mPortValueMap;

  struct MatrixDumper : public Referenced {
    MatrixDumper(const NumericPortValue* numericPortValue,
                 const HDF5Object& parent, const std::string& name) :
      mNumericPortValue(numericPortValue),
      _stream(parent, name)
    { OpenFDMAssert(numericPortValue); }
    void append()
    { _stream.append(mNumericPortValue->getValue()); }

    SharedPtr<const NumericPortValue> mNumericPortValue;
    HDFMatrixStream _stream;
  };

  typedef std::list<SharedPtr<MatrixDumper> > DumperList;
  DumperList mDumperList;
};

} // namespace OpenFDM

#endif
