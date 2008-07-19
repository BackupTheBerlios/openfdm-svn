/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_HDF5Writer_H
#define OpenFDM_HDF5Writer_H

#include <sstream>
#include <set>
#include <hdf5.h>
#include <OpenFDM/ModelGroup.h>
#include <OpenFDM/ModelVisitor.h>
#include <OpenFDM/System.h>

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

class HDF5Writer : public ModelVisitor {
public:
  HDF5Writer(const std::string& filename) :
    _hdf5File(filename),
    _group(_hdf5File, "System"),
    _ts(_group.getId(), "t")
  { }
  ~HDF5Writer()
  { }

  virtual void apply(Model& model)
  {
    HDF5Group modelGroup(_group, model.getName());

    dumpObject(modelGroup, model);

    unsigned numOutputs = model.getNumOutputPorts();
    if (numOutputs > 0) {
      HDF5Group outputGroup(modelGroup, "outputs");
      UniqueStringSet outputStringSet;
      for (unsigned i = 0; i < numOutputs; ++i) {
        NumericPortProvider* numericPort = model.getOutputPort(i);
        if (!numericPort)
          continue;
        PortInterface* portInterface = numericPort->getPortInterface();
        if (!portInterface)
          continue;
        MatrixPortInterface* matrixPortInterface;
        matrixPortInterface = portInterface->toMatrixPortInterface();
        if (!matrixPortInterface)
          continue;
        
        std::string name;
        name = outputStringSet.makeUnique(model.getOutputPort(i)->getName());
        _dumperList.push_back(new MatrixDumper(matrixPortInterface, outputGroup.getId(), name));
      }
    }
  }
  virtual void apply(ModelGroup& modelGroup)
  {
    HDF5Group parentGroup = _group;
    _group = HDF5Group(_group, modelGroup.getName()),

    dumpObject(_group, modelGroup);

    traverse(modelGroup);
    _group = parentGroup;
  }
  virtual void apply(System& system)
  {
    if (_dumperList.empty())
      ModelVisitor::apply(system);
    _ts.append(system.getTime());
    for (DumperList::iterator i = _dumperList.begin(); i != _dumperList.end();
         ++i) {
      (*i)->append();
    }
  }

private:
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

  void dumpObject(HDF5Object& parent, Object& object)
  {
    HDF5Group propertyGroup(parent, "properties");
    std::vector<PropertyInfo> propertyInfoList;
    object.getPropertyInfoList(propertyInfoList);
    std::vector<PropertyInfo>::const_iterator i;
    for (i = propertyInfoList.begin(); i != propertyInfoList.end(); ++i) {
      std::string name = i->getName();
      Variant v;
      if (!object.getPropertyValue(name, v))
        continue;
      if (v.isMatrix()) {
        HDFMatrixStream value(propertyGroup, name);
        value.append(v.toMatrix());
      } if (v.isReal()) {
        HDFMatrixStream value(propertyGroup, name);
        value.append(v.toReal());
      } if (v.isInteger()) {
        // FIXME, converts to double ...
        HDFMatrixStream value(propertyGroup, name);
        value.append(v.toInteger());
      } if (v.isUnsigned()) {
        // FIXME, converts to double ...
        HDFMatrixStream value(propertyGroup, name);
        value.append(v.toUnsigned());
      }
    }
  }

  HDF5File _hdf5File;
  HDF5Group _group;

  HDFMatrixStream _ts;

  struct MatrixDumper : public Referenced {
    MatrixDumper(MatrixPortInterface* matrixPortInterface, const HDF5Object& parent, const std::string& name) :
      _matrixPortInterface(matrixPortInterface),
      _stream(parent, name)
    { }
    void append()
    { _stream.append(_matrixPortInterface->getMatrixValue()); }

    SharedPtr<MatrixPortInterface> _matrixPortInterface;
    HDFMatrixStream _stream;
  };

  typedef std::list<SharedPtr<MatrixDumper> > DumperList;
  DumperList _dumperList;
};

} // namespace OpenFDM

#endif
