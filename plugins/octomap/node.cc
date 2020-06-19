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

      octomap::OcTreeNode* node(tree.getRoot());
      if(node == NULL)
        throw std::invalid_argument ("OcTree empty");

      std::cout<<"tree volume: "<<tree.volume()<<std::endl;
      
      std::cout<<"root size: "<<tree.getNodeSize(0)<<std::endl;

      std::cout<<"tree depth: "<<tree.getTreeDepth()<<std::endl;

      std::cout<<"number of nodes: "<<tree.calcNumNodes()<<std::endl;

      auto coucou = tree.begin();

      for(int levelidx = 0; levelidx <= tree.getTreeDepth(); levelidx++)
        std::cout<<"node at level "<<levelidx<<"of size "<<tree.getNodeSize(levelidx)<<std::endl;
      // recursivedisplay(&tree, node, 0, 2);

      int i = 0;
      int size2 = 0;
      int size3 = 0;
      int size6 = 0;
      int size12 = 0;
      int size24 = 0;
      int size48 = 0;
      int size96 = 0;
      int size192 = 0;
      int size384 = 0;
      int size768 = 0;
      int size1536 = 0;
      int size3072 = 0;
      int size6144 = 0;
      int size12288 = 0;
      int size24576 = 0;
      int size49152 = 0;
      int size98304 = 0;
      for(auto it = tree.begin_leafs(), end=tree.end_leafs(); it!= end; ++it)
      {
        i++;
        //manipulate node, e.g.:
        // std::cout << "Node center: " << it.getCoordinate() << std::endl;
        // std::cout << "Node size: " << it.getSize() << std::endl;
        // std::cout << "Node value: " << it->getValue() << std::endl;
        // auto coordinates = it.getCoordinate();
        // addCube(it.getSize(), osg::Vec3(coordinates.x(), coordinates.x(), coordinates.x()));
        if(it.getSize() == 0.02){
          size2++;
        } else if(it.getSize() == 0.03){ size3++;
        } else if(it.getSize() == 0.06){ size6++;
        } else if(it.getSize() == 0.12){ size12++;
        } else if(it.getSize() == 0.24){ size24++;
        } else if(it.getSize() == 0.48){ size48++;
        } else if(it.getSize() == 0.96){ size96++;
        } else if(it.getSize() == 1.92){ size192++;
        } else if(it.getSize() == 3.84){ size384++;
        } else if(it.getSize() == 7.68){ size768++;
        } else if(it.getSize() == 15.36){ size1536++;
        } else if(it.getSize() == 30.72){ size3072++;
        } else if(it.getSize() == 61.44){ size6144++;
        } else if(it.getSize() == 122.88){ size12288++;
        } else if(it.getSize() == 245.76){ size24576++;
        } else if(it.getSize() == 491.52){ size49152++;
        } else {
          std::cout<<"uncounted size: "<<it.getSize()<<std::endl;
        }
      }
      std::cout<<"number of leafs: "<<i<<std::endl;
      std::cout<<"number of 0.02 leafs: "<<size2<<std::endl;
      std::cout<<"number of 0.03 leafs: "<<size3<<std::endl;
      std::cout<<"number of 0.06 leafs: "<<size6<<std::endl;
      std::cout<<"number of 0.12 leafs: "<<size12<<std::endl;
      std::cout<<"number of 0.24 leafs: "<<size24<<std::endl;
      std::cout<<"number of 0.48 leafs: "<<size48<<std::endl;
      std::cout<<"number of 0.96 leafs: "<<size96<<std::endl;
      std::cout<<"number of 1.92 leafs: "<<size192<<std::endl;
      std::cout<<"number of 3.84 leafs: "<<size384<<std::endl;
      std::cout<<"number of 7.68 leafs: "<<size768<<std::endl;
      std::cout<<"number of 15.36 leafs: "<<size1536<<std::endl;
      std::cout<<"number of 30.72 leafs: "<<size3072<<std::endl;
      std::cout<<"number of 61.44 leafs: "<<size6144<<std::endl;
      std::cout<<"number of 122.88 leafs: "<<size12288<<std::endl;
      std::cout<<"number of 245.76 leafs: "<<size24576<<std::endl;
      std::cout<<"number of 491.52 leafs: "<<size49152<<std::endl;
  // template <class NODE,class I>
  // void OcTreeBaseImpl<NODE,I>::calcNumNodesRecurs(NODE* node, size_t& num_nodes) const {
  //   assert (node);
  //   if (nodeHasChildren(node)) {
  //     for (unsigned int i=0; i<8; ++i) {
  //       if (nodeChildExists(node, i)) {
  //         num_nodes++;
  //         calcNumNodesRecurs(getNodeChild(node, i), num_nodes);
  //       }
  //     }
  //   }
  // }
      addCube(0.1, osg::Vec3(0., 0., 0.));

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
