/* -*-c++-*- OpenFDM - Copyright (C) 2005-2009 Mathias Froehlich 
 *
 */

#include "ModelItem.h"

#include <iostream>
#include <OpenFDM/Model.h>
#include <OpenFDM/ModelGroup.h>

using OpenFDM::Model;
using OpenFDM::ModelGroup;

static Model* modelFrom(const QModelIndex& index)
{
  if (!index.isValid())
    return 0;
  return static_cast<Model*>(index.internalPointer());
}

static ModelGroup* modelGroupFrom(const QModelIndex& index)
{
  if (!index.isValid())
    return 0;
  Model* model = static_cast<Model*>(index.internalPointer());
  if (!model)
    return 0;
  return model->toModelGroup();
}

ModelItem::ModelItem(QObject* parent) :
  QAbstractItemModel(parent)
{
}

ModelItem::~ModelItem(void)
{
}

QModelIndex
ModelItem::index(int row, int column, const QModelIndex &parent) const
{
  if (!parent.isValid()) {
    if (!mSystem)
      return QModelIndex();
    if (row != 0)
      return QModelIndex();

    return createIndex(row, 0, mSystem);
  }

  ModelGroup* modelGroup = modelGroupFrom(parent);
  if (!modelGroup)
    return QModelIndex();
  if (modelGroup->getNumModels() <= row)
    return QModelIndex();

  Model* childModel = modelGroup->getModel(row);
  if (!childModel)
    return QModelIndex();

  return createIndex(row, 0, childModel);
}

QModelIndex
ModelItem::parent(const QModelIndex &child) const
{
  if (!child.isValid())
    return QModelIndex();

  Model* model = modelFrom(child);
  if (!model)
    return QModelIndex();

  Model* parentModel = model->getParent();
  if (!parentModel)
    return QModelIndex();
  if (parentModel == mSystem)
    return createIndex(0, 0, parentModel);

  Model* parentModel2 = parentModel->getParent();
  if (!parentModel2)
    return QModelIndex();
  ModelGroup* parentModelGroup2 = parentModel2->toModelGroup();
  if (!parentModelGroup2)
    return QModelIndex();
  unsigned row = parentModelGroup2->getModelIndex(parentModel);
  if (row >= parentModelGroup2->getNumModels())
    return QModelIndex();

  return createIndex(row, 0, parentModel);
}

int
ModelItem::rowCount(const QModelIndex &parent) const
{
  if (!parent.isValid()) {
    if (!mSystem)
      return 0;

    return 1;
  }

  ModelGroup* modelGroup = modelGroupFrom(parent);
  if (!modelGroup)
    return 0;
  return modelGroup->getNumModels();
}

int
ModelItem::columnCount(const QModelIndex &parent) const
{
  return 2;
}

bool
ModelItem::hasChildren(const QModelIndex &parent) const
{
  if (!parent.isValid())
    return true;
  ModelGroup* modelGroup = modelGroupFrom(parent);
  if (!modelGroup)
    return false;
  return 0 < modelGroup->getNumModels();
}

QVariant
ModelItem::data(const QModelIndex &index, int role) const
{
  if (!index.isValid())
    return QVariant();

  Model* model = modelFrom(index);
  if (!model)
    return QVariant();
  switch (role) {
  case Qt::DisplayRole:
    return QVariant(QString(model->getName().c_str()));
  case Qt::ToolTipRole:
    return QVariant(QString(model->getTypeName()));
  default:
    return QVariant();
  }
}

QVariant
ModelItem::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
    return QVariant();

  switch (section) {
  case 0:
    return QString("Name");
  case 1:
    return QString("Properties");
  default:
    return QVariant();
  }
}

Qt::ItemFlags
ModelItem::flags(const QModelIndex &index) const
{
//   if (!index.isValid())
//     return Qt::ItemIsEnabled;

  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}
