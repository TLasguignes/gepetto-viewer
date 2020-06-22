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

#include <node.hh>

#include <osg/Version>

namespace gepetto {
  namespace gui {
    OcTreeDisplay::OcTreeDisplay (const std::string& filename, const std::string& name)
      : Node (name)
      , filename_ (filename)
    {}

    void OcTreeDisplay::setColor (const osgVector4& color)
    {
      for (auto& drawable : boxes) {
        drawable->setColor(color);
#if OSG_VERSION_GREATER_OR_EQUAL(3, 5, 6)
        drawable->build();
#else
        drawable->dirtyDisplayList();
        drawable->dirtyBound();
#endif
      }
      setDirty();
    }

    void OcTreeDisplay::recursivedisplay(octomap::OcTree* ptree, octomap::OcTreeNode* node, int level, int show_level){
      if(level <= show_level){
        if(ptree->nodeHasChildren(node)){
          for(unsigned int i=0; i<8; ++i){
            if(ptree->nodeChildExists(node, i)){
            }
          }
        }
      } else {
      }
    }

    void OcTreeDisplay::init ()
    {
      if (!geode) geode = new ::osg::Geode;

      // TODO required
      // Create octree from member filename_
      // and loop on the leaf boxes
      octomap::OcTree tree(filename_);

      for(octomap::OcTree::tree_iterator tree_it = tree.begin_tree(tree.getTreeDepth()), tree_end = tree.end_tree(); tree_it != tree_end; tree_it++){
        if(tree_it.isLeaf()){
          if(tree.isNodeOccupied(*tree_it)){
            octomap::point3d coordinates = tree_it.getCoordinate();
            addCube(tree_it.getSize(), osg::Vec3(coordinates.x(), coordinates.y(), coordinates.z()));
          }
        }
      }

      this->asQueue()->addChild(geode);
    }

    void OcTreeDisplay::addCube (float size, osg::Vec3 origin)
    {
      ::osg::BoxRefPtr box = new ::osg::Box ();
      box->setCenter      (origin);
      box->setHalfLengths (osg::Vec3 (size/2,size/2,size/2));

      ::osg::ShapeDrawableRefPtr drawable = new ::osg::ShapeDrawable(box);
      boxes.push_back (drawable);
      geode->addDrawable(drawable);
    }
  } // namespace gui
} // namespace gepetto
