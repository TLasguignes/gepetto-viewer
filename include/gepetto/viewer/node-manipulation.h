//
//  node-visitor.h
//  gepetto-viewer
//
//  Created by Joseph Mirabel in 2016.
//  Copyright (c) 2016 LAAS-CNRS. All rights reserved.
//

#ifndef SCENEVIEWER_NODE_MANIPULATOR_HH
#define SCENEVIEWER_NODE_MANIPULATOR_HH

namespace osg {
  class Node;
}

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

    osg::Node* addDraggerToScene(osg::Node* scene, const DraggerType& type, bool fixedSizeInScreen);
  } /* namespace nodeManipulation */
} /* namespace graphics */

#endif /* SCENEVIEWER_NODE_MANIPULATOR_HH */
