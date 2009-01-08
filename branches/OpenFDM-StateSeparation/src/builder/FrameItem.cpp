/* -*-c++-*- OpenFDM - Copyright (C) 2005-2009 Mathias Froehlich 
 *
 */

#include "FrameItem.h"

#include <iostream>
#include <OpenFDM/Frame.h>

using OpenFDM::Frame;

static Frame* frameFrom(const QModelIndex& index)
{
  if (!index.isValid())
    return 0;
  return static_cast<Frame*>(index.internalPointer());
}

FrameItem::FrameItem(QObject* parent) :
  QAbstractItemModel(parent)
{
}

FrameItem::~FrameItem(void)
{
}

QModelIndex
FrameItem::index(int row, int column, const QModelIndex &parent) const
{
  if (!parent.isValid()) {
    if (!mRootFrame)
      return QModelIndex();
    if (row != 0)
      return QModelIndex();

    return createIndex(row, 0, mRootFrame);
  }

  Frame* frame = frameFrom(parent);
  if (!frame)
    return QModelIndex();
  if (frame->getNumChildFrames() <= row)
    return QModelIndex();

  Frame* childFrame = frame->getChildFrame(row);
  if (!childFrame)
    return QModelIndex();

  return createIndex(row, 0, childFrame);
}

QModelIndex
FrameItem::parent(const QModelIndex &child) const
{
  if (!child.isValid())
    return QModelIndex();

  Frame* frame = frameFrom(child);
  if (!frame)
    return QModelIndex();

  Frame* parentFrame = frame->getParentFrame();
  if (!parentFrame)
    return QModelIndex();
  if (parentFrame == mRootFrame)
    return createIndex(0, 0, parentFrame);

  Frame* parentFrame2 = parentFrame->getParentFrame();
  if (!parentFrame2)
    return QModelIndex();
  unsigned row = parentFrame2->getChildFrameIndex(parentFrame);
  if (row >= parentFrame2->getNumChildFrames())
    return QModelIndex();

  return createIndex(row, 0, parentFrame);
}

int
FrameItem::rowCount(const QModelIndex &parent) const
{
  if (!parent.isValid()) {
    if (!mRootFrame)
      return 0;

    return 1;
  }

  Frame* frame = frameFrom(parent);
  if (!frame)
    return 0;
  return frame->getNumChildFrames();
}

int
FrameItem::columnCount(const QModelIndex &parent) const
{
  return 2;
}

bool
FrameItem::hasChildren(const QModelIndex &parent) const
{
  if (!parent.isValid())
    return true;
  Frame* frame = frameFrom(parent);
  if (!frame)
    return false;
  return 0 < frame->getNumChildFrames();
}

QVariant
FrameItem::data(const QModelIndex &index, int role) const
{
  if (!index.isValid())
    return QVariant();

  Frame* frame = frameFrom(index);
  if (!frame)
    return QVariant();
  switch (role) {
  case Qt::DisplayRole:
    return QVariant(QString(frame->getName().c_str()));
  case Qt::ToolTipRole:
    return QVariant(QString(frame->getTypeName()));
  default:
    return QVariant();
  }
}

QVariant
FrameItem::headerData(int section, Qt::Orientation orientation, int role) const
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
FrameItem::flags(const QModelIndex &index) const
{
//   if (!index.isValid())
//     return Qt::ItemIsEnabled;

  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}
