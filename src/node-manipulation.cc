/* OpenSceneGraph example, osgmanipulator.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

#include <gepetto/viewer/node-manipulation.h>

#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabBoxTrackballDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>

#include <osg/MatrixTransform>

namespace graphics {
  namespace nodeManipulation {
    template <typename DraggerDerived> DraggerDerived* newDragger () {
      DraggerDerived* d = new DraggerDerived();
      d->setupDefaultGeometry();
      return d;
    }

    osgManipulator::Dragger* createDragger(const DraggerType& type)
    {
      osgManipulator::Dragger* dragger = 0;
      switch (type) {
        case TabPlaneDragger:
          dragger = newDragger<osgManipulator::TabPlaneDragger>();
          break;
        case TabPlaneTrackballDragger:
          dragger = newDragger<osgManipulator::TabPlaneTrackballDragger>();
          break;
        case TabBoxTrackballDragger:
          dragger = newDragger<osgManipulator::TabBoxTrackballDragger>();
          break;
        case TrackballDragger:
          dragger = newDragger<osgManipulator::TrackballDragger>();
          break;
        case Translate1DDragger:
          dragger = newDragger<osgManipulator::Translate1DDragger>();
          break;
        case Translate2DDragger:
          dragger = newDragger<osgManipulator::Translate2DDragger>();
          break;
        case TranslateAxisDragger:
          dragger = newDragger<osgManipulator::TranslateAxisDragger>();
          break;
        case TabBoxDragger:
          dragger = newDragger<osgManipulator::TabBoxDragger>();
          break;
      };
      return dragger;
    }

    // The DraggerContainer node is used to fix the dragger's size on the screen
    class DraggerContainer : public osg::Group
    {
      public:
        DraggerContainer() : _draggerSize(240.0f), _active(true) {}

        DraggerContainer( const DraggerContainer& copy, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY )
          :   osg::Group(copy, copyop),
          _dragger(copy._dragger), _draggerSize(copy._draggerSize), _active(copy._active)
        {}

        META_Node( osgManipulator, DraggerContainer );

        void setDragger( osgManipulator::Dragger* dragger )
        {
          _dragger = dragger;
          if ( !containsNode(dragger) ) addChild( dragger );
        }

        osgManipulator::Dragger* getDragger() { return _dragger.get(); }
        const osgManipulator::Dragger* getDragger() const { return _dragger.get(); }

        void setDraggerSize( float size ) { _draggerSize = size; }
        float getDraggerSize() const { return _draggerSize; }

        void setActive( bool b ) { _active = b; }
        bool getActive() const { return _active; }

        void traverse( osg::NodeVisitor& nv )
        {
          if ( _dragger.valid() )
          {
            if ( _active && nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
            {
              osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);

              float pixelSize = cv->pixelSize(_dragger->getBound().center(), 0.48f);
              if ( pixelSize!=_draggerSize )
              {
                float pixelScale = pixelSize>0.0f ? _draggerSize/pixelSize : 1.0f;
                osg::Vec3d scaleFactor(pixelScale, pixelScale, pixelScale);

                osg::Vec3 trans = _dragger->getMatrix().getTrans();
                _dragger->setMatrix( osg::Matrix::scale(scaleFactor) * osg::Matrix::translate(trans) );
              }
            }
          }
          osg::Group::traverse(nv);
        }

      protected:
        osg::ref_ptr<osgManipulator::Dragger> _dragger;
        float _draggerSize;
        bool _active;
    };

    osg::Node* addDraggerToScene(osg::Node* scene, const DraggerType& type, bool fixedSizeInScreen)
    {
      scene->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

      osg::MatrixTransform* selection = new osg::MatrixTransform;
      selection->addChild(scene);

      osgManipulator::Dragger* dragger = createDragger(type);

      osg::Group* root = new osg::Group;
      root->addChild(selection);

      if ( fixedSizeInScreen )
      {
        DraggerContainer* draggerContainer = new DraggerContainer;
        draggerContainer->setDragger( dragger );
        root->addChild(draggerContainer);
      }
      else
        root->addChild(dragger);

      float scale = scene->getBound().radius() * 1.6f;
      dragger->setMatrix(osg::Matrix::scale(scale, scale, scale) *
          osg::Matrix::translate(scene->getBound().center()));

      dragger->addTransformUpdating(selection);

      // we want the dragger to handle it's own events automatically
      dragger->setHandleEvents(true);

      // if we don't set an activation key or mod mask then any mouse click on
      // the dragger will activate it, however if do define either of ActivationModKeyMask or
      // and ActivationKeyEvent then you'll have to press either than mod key or the specified key to
      // be able to activate the dragger when you mouse click on it.  Please note the follow allows
      // activation if either the ctrl key or the 'a' key is pressed and held down.
      dragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
      //dragger->setActivationKeyEvent('a');

      return root;
    }
  } /* namespace nodeManipulation */
} /* namespace graphics */
