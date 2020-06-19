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

#ifndef HPP_GUI_OCTOMAP_PLUGIN_NODE_HH
#define HPP_GUI_OCTOMAP_PLUGIN_NODE_HH

#include <gepetto/viewer/node.h>

#include <octomap/OcTree.h>

namespace gepetto {
  namespace gui {
    typedef gepetto::viewer::Node Node;

    DEF_CLASS_SMART_PTR(OcTreeDisplay)

    class OcTreeDisplay : public Node
    {
      public:
        OcTreeDisplay (const std::string& filename, const std::string& name);

        void setColor (const osgVector4& color);

        void init ();

      private:
        /// \param size the length of each side of the cube.
        void addCube (float size, osg::Vec3 origin);

        /// \param the main octree to explore
        /// \param the node to explore childrens
        /// \param the node depth level
        /// \param the depth level to show
        void recursivedisplay(octomap::OcTree* ptree, octomap::OcTreeNode* node, int level, int show_level);

        std::vector< ::osg::ShapeDrawableRefPtr > boxes;
        ::osg::GeodeRefPtr geode;

        const std::string filename_;
    };
  } // namespace gui
} // namespace gepetto

#endif // HPP_GUI_OCTOMAP_PLUGIN_NODE_HH
