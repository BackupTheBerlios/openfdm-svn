/* -*-c++-*- OpenFDM - Copyright (C) 2005-2009 Mathias Froehlich 
 *
 */

#ifndef FrameItem_H
#define FrameItem_H

#include <QtCore/QAbstractItemModel>

#include <OpenFDM/System.h>
#include <OpenFDM/Frame.h>

class FrameItem : public QAbstractItemModel {
public:
  FrameItem(QObject* parent = 0);
  virtual ~FrameItem(void);

  virtual QModelIndex index(int row, int col, const QModelIndex &parent) const;
  virtual QModelIndex parent(const QModelIndex &child) const;

  virtual int rowCount(const QModelIndex &parent) const;
  virtual int columnCount(const QModelIndex &parent) const;
  virtual bool hasChildren(const QModelIndex &parent) const;

  virtual QVariant data(const QModelIndex &index, int role) const;
//virtual bool setData(const QModelIndex &index, const QVariant &value, int role);

  virtual QVariant headerData(int section, Qt::Orientation orientation,
                              int role) const;
//virtual bool setHeaderData(int section, Qt::Orientation orientation, const QVariant &value,
//                                int role);

//     virtual QMap<int, QVariant> itemData(const QModelIndex &index) const;
//     virtual bool setItemData(const QModelIndex &index, const QMap<int, QVariant> &roles);

//     virtual QStringList mimeTypes() const;
//     virtual QMimeData *mimeData(const QModelIndexList &indexes) const;
//     virtual bool dropMimeData(const QMimeData *data, Qt::DropAction action,
//                               int row, int col, const QModelIndex &parent);
//     virtual Qt::DropActions supportedDropActions() const;

//     virtual bool insertRows(int row, int count, const QModelIndex &parent);
//     virtual bool insertColumns(int col, int count, const QModelIndex &parent);
//     virtual bool removeRows(int row, int count, const QModelIndex &parent);
//     virtual bool removeColumns(int col, int count, const QModelIndex &parent);

//     inline bool insertRow(int row, const QModelIndex &parent);
//     inline bool insertColumn(int col, const QModelIndex &parent);
//     inline bool removeRow(int row, const QModelIndex &parent);
//     inline bool removeColumn(int col, const QModelIndex &parent);

//     virtual void fetchMore(const QModelIndex &parent);
//     virtual bool canFetchMore(const QModelIndex &parent) const;
  virtual Qt::ItemFlags flags(const QModelIndex &index) const;
//     virtual void sort(int col, Qt::SortOrder order);
//     virtual QModelIndex buddy(const QModelIndex &index) const;
//     virtual QModelIndexList match(const QModelIndex &start, int role,
//                                   const QVariant &value, int hits,
//                                   Qt::MatchFlags flags) const;
//     virtual QSize span(const QModelIndex &index) const;

  /// set and get the system represented with that ItemModel
  void setRootFrame(OpenFDM::Frame* frame)
  { mRootFrame = frame; }
  OpenFDM::Frame* getRootFrame(void) const
  { return mRootFrame; }

private:
  OpenFDM::SharedPtr<OpenFDM::Frame> mRootFrame;
};

#endif
