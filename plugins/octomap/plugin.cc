// Copyright (c) 2019 CNRS
// Authors: Joseph Mirabel
//
//
// This file is part of hpp-gui
// hpp-gui is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-gui is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-gui  If not, see
// <http://www.gnu.org/licenses/>.

#include <plugin.hh>

#include <QToolBar>
#include <QAction>
#include <QFileDialog>
#include <QInputDialog>

#include <gepetto/viewer/window-manager.h>
#include <gepetto/gui/mainwindow.hh>
#include <gepetto/gui/osgwidget.hh>
#include <gepetto/gui/windows-manager.hh>

#include <node.hh>

namespace gepetto {
  namespace gui {
    using gepetto::gui::MainWindow;

    void OctomapPlugin::init()
    {
      MainWindow* main = MainWindow::instance ();
      main->registerSlot("addOcTree", this);

      // TODO add a way to add an action to body tree items.
      QToolBar* toolBar = MainWindow::instance()->addToolBar("OcTree tools");
      toolBar->setObjectName ("octomapplugin.toolbar");
      QAction* openD = new QAction (QIcon::fromTheme("document-open"), "Load a BVH model", toolBar);
      toolBar->addAction (openD);
      connect (openD, SIGNAL(triggered()), SLOT (openDialog()));
    }

    void OctomapPlugin::addOcTree (QString name, QString filename) const
    {
      std::string _name (name.toStdString());

      OcTreeDisplayPtr_t node (new OcTreeDisplay (filename.toStdString(), _name));
      node->init ();

      MainWindow* main = MainWindow::instance ();
      main->osg()->insertNode (_name, node);
    }

    void OctomapPlugin::openDialog() const
    {
      QString filename = QFileDialog::getOpenFileName (NULL, "Select a mesh file");
      if (filename.isNull()) return;
      QString name = QInputDialog::getText(NULL, "Node name", "Node name", QLineEdit::Normal, "octree");
      if (name.isNull()) return;

      std::string filename_ (filename.toStdString());
      std::string name_ (name.toStdString());

      MainWindow* main = MainWindow::instance ();

      std::string group;
      if (main->osgWindows().empty()) {
        group = "window";
        main->osg()->createWindow (group);
      } else {
        group = main->osgWindows().first()->window()->getID();
      }

      addOcTree (name, filename);
      main->osg()->addToGroup (name_, group);
    }

#if (QT_VERSION < QT_VERSION_CHECK(5,0,0))
    Q_EXPORT_PLUGIN2 (octomapplugin, OctomapPlugin)
#endif

  } // namespace gui
} // namespace gepetto
