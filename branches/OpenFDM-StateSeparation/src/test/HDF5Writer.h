/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_HDF5Writer_H
#define OpenFDM_HDF5Writer_H

#include <sstream>
#include <set>
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
  { return H5Iget_ref(_id); }
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
  void assign(hid_t id)
  {
    if (_id == id)
      return;
    if (_id < 0 || H5I_BADID == H5Iget_type(_id))
      release();
    H5Iinc_ref(id);
    release();
    _id = id;
  }
// private:
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
#if (1 < H5_VERS_MAJOR || (1 == H5_VERS_MAJOR && 8 <= H5_VERS_MINOR))
    _id = H5Gcreate(parent.getId(), name.c_str(),  H5P_DEFAULT,  H5P_DEFAULT,  H5P_DEFAULT);
#else
    _id = H5Gcreate(parent.getId(), name.c_str(), 0);
#endif
    if (_id < 0)
      return false;
    return true;
  }
};

class HDF5File : public HDF5Object {
public:
  HDF5File()
  { }
  HDF5File(const std::string& filename)
  { open(filename); }
  void open(const std::string& name)
  { _id = H5Fcreate(name.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT); }
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
    if (_id < 0) {
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
#if (1 < H5_VERS_MAJOR || (1 == H5_VERS_MAJOR && 8 <= H5_VERS_MINOR))
      _id = H5Dcreate(_parent.getId(), _name.c_str(), H5T_NATIVE_DOUBLE,
                      _dataspace.getId(), H5P_DEFAULT, cparms, H5P_DEFAULT);
#else
      _id = H5Dcreate(_parent.getId(), _name.c_str(), H5T_NATIVE_DOUBLE,
                      _dataspace.getId(), cparms);
#endif
      H5Pclose(cparms);
    } else {
      // increment size
      _dims[0] += 1;
      status = H5Dextend(_id, _dims);
    }

    HDF5Object filespace = H5Dget_space(_id);
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




class HDF5Log : protected ConstNodeVisitor {
public:
  HDF5Log(const std::string& filename) :
    mHDF5File(filename),
    mCurrentGroup(mHDF5File, "System"),
    mTimeStream(mCurrentGroup.getId(), "t")
  { }
  ~HDF5Log()
  { }

  void attachTo(const System* system)
  {
    if (!system)
      return;
    if (!system->getNode())
      return;
    mSystem = system;
    system->getNode()->accept(*this);
  }

  void dump()
  {
    if (mSystem)
      mTimeStream.append(mSystem->getTime());
    DumperList::iterator i;
    for (i = mDumperList.begin(); i != mDumperList.end(); ++i)
      (*i)->append();
  }

private:
  const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const
  {
    /// FIXME, use a map for that???
    NodeInstanceList::const_iterator i;
    for (i = mSystem->getNodeInstanceList().begin();
         i != mSystem->getNodeInstanceList().end(); ++i) {
      if ((*i)->getNodePath() == nodePath)
        return (*i);
    }
    return 0;
  }

  void appendPortValues(const AbstractNodeInstance& nodeInstance)
  {
    UniqueStringSet uniqueStringSet;

    HDF5Group portValuesGroup(mCurrentGroup, "portValues");
    unsigned numPorts = nodeInstance.getNode().getNumPorts();
    for (unsigned i = 0; i < numPorts; ++i) {
      const PortValue* portValue;
      portValue = nodeInstance.getPortValueList().getPortValue(i);
      const NumericPortValue* npv;
      npv = dynamic_cast<const NumericPortValue*>(portValue);
      if (!npv)
        continue;
      std::string name = nodeInstance.getNode().getPort(i)->getName();
      name = uniqueStringSet.makeUnique(name);
      mDumperList.push_back(new MatrixDumper(npv, portValuesGroup, name));
    }
  }
  void appendPortValues(const Node& node)
  {
    const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
    OpenFDMAssert(nodeInstance);
    appendPortValues(*nodeInstance);
  }

  virtual void apply(const Node& node)
  {
    HDF5Group parentGroup = mCurrentGroup;
    mCurrentGroup = HDF5Group(parentGroup, node.getName());
    appendPortValues(node);
    mCurrentGroup = parentGroup;
  }
  virtual void apply(const Group& group)
  {
    HDF5Group parentGroup = mCurrentGroup;
    mCurrentGroup = HDF5Group(parentGroup, group.getName());

    appendPortValues(group);
    group.traverse(*this);

    mCurrentGroup = parentGroup;
  }

  // FIXME: do we need???
  SharedPtr<const System> mSystem;

  HDF5File mHDF5File;
  HDF5Group mCurrentGroup;
  HDFMatrixStream mTimeStream;

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

  // Helper class that makes names unique ...
  struct UniqueStringSet {
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
  private:
    std::set<std::string> _strings;
  };
};

} // namespace OpenFDM

#endif
