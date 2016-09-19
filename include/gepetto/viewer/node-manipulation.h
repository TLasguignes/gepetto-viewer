//
//  node-visitor.h
//  gepetto-viewer
//
//  Created by Joseph Mirabel in 2016.
//  Copyright (c) 2016 LAAS-CNRS. All rights reserved.
//

#ifndef SCENEVIEWER_NODE_MANIPULATOR_HH
#define SCENEVIEWER_NODE_MANIPULATOR_HH

#include <osgManipulator/Dragger>

namespace graphics {
  namespace nodeManipulation {
    enum DraggerType {
      TabPlaneDragger,
      TabPlaneTrackballDragger,
      TabBoxTrackballDragger,
      TrackballDragger,
      Translate1DDragger,
      Translate2DDragger,
      TranslateAxisDragger,
      TabBoxDragger
    };

    class NodeDragger {
      public:
        ::osg::ref_ptr<osgManipulator::Dragger> dragger_;
        ::osg::ref_ptr<osg::Group> scene_;
        ::osg::ref_ptr<osg::MatrixTransform> selection_;

        NodeDragger(float scaling = 1.6f);

        bool empty ();

        void addDragger (const DraggerType& type, bool fixedSizeInScreen);

        void setSceneAndTransform (::osg::Group* scene, ::osg::MatrixTransform* selection);

        void removeDragger ();

      private:
        float scaling_;
        ::osg::Node* rootChild_;
    };
  } /* namespace nodeManipulation */
} /* namespace graphics */

#endif /* SCENEVIEWER_NODE_MANIPULATOR_HH */
