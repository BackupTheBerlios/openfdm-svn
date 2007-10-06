/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelGroup_H
#define OpenFDM_ModelGroup_H

#include <vector>

#include "Object.h"
#include "Model.h"
#include "NumericPortProxy.h"

namespace OpenFDM {

class Joint;

class ModelGroup : public Node {
  OPENFDM_OBJECT(ModelGroup, Node);
public:
  ModelGroup(const std::string& name);
  virtual ~ModelGroup(void);

  /// Double dispatch helper for the ModelVisitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;
  /// Traverse this ModelGroup with the given visitor
  void traverse(ModelVisitor& visitor);

  virtual const ModelGroup* toModelGroup(void) const;
  virtual ModelGroup* toModelGroup(void);

  /// Interfaces to gain access to the Models contained in this ModelGroup
  unsigned getNumModels(void) const { return mModels.size(); }
  const Node* getModel(unsigned idx) const;
  Node* getModel(unsigned idx);
  const Node* getModel(const std::string& name) const;
  Node* getModel(const std::string& name);
  unsigned getModelIndex(const std::string& name) const;
  unsigned getModelIndex(const Node* const model) const;
  unsigned addModel(Node* model, bool allowRename = false);
  void removeModel(Node* model);

  /// Interfaces to gain access to the Connections contained in this ModelGroup
  void addConnection(Connection* connection);
  void removeConnection(Connection* connection);
  unsigned getConnectionIndex(const Connection* const connection) const;
  unsigned getNumConnections(void) const { return mConnections.size(); }
  Connection* getConnection(unsigned i);
  const Connection* getConnection(unsigned i) const;

  /// Returns the path of this ModelGroup. In contrast to getPath this returns
  /// the path including the current ModelGroup.
  Path getGroupPath() /* FIXME const*/;

  bool addSampleTime(const SampleTime& sampleTime)
  { return mSampleTimeSet.addSampleTime(sampleTime); }
  bool removeSampleTime(const SampleTime& sampleTime)
  { return mSampleTimeSet.removeSampleTime(sampleTime); }
  const SampleTimeSet& getSampleTimeSet(void) const
  { return mSampleTimeSet; }

private:

  typedef std::vector<SharedPtr<Node> > ModelList;
  typedef std::vector<SharedPtr<Connection> > ConnectionList;

  /// The List of models contained in this group.
  ModelList mModels;
  SampleTimeSet mSampleTimeSet;
  /// The list of connections lines within this model group
  ConnectionList mConnections;
};

} // namespace OpenFDM

#endif
