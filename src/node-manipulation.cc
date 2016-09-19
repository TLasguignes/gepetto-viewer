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
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>

#include <osg/MatrixTransform>

#include <iostream>

// TODO This is dirty...
// #include <node-manipulation/tab-plane-dragger.cc>
#include <../src/node-manipulation/tab-plane-dragger.cc>

namespace graphics {
  namespace nodeManipulation {
    typedef TabPlaneDraggerTpl<0,1> TabPlaneDraggerXY;

    template <typename DraggerDerived> DraggerDerived* newDragger () {
      DraggerDerived* d = new DraggerDerived();
      d->setupDefaultGeometry();
      return d;
    }
    template <> TabPlaneDraggerXY* newDragger<TabPlaneDraggerXY> () {
      TabPlaneDraggerXY* d = new TabPlaneDraggerXY();
      // std::cout << d->getMatrix() << std::endl;
      // d->setMatrix(osg::Matrix::rotate(osg::Vec3f(0.f,0.f,1.f), osg::Vec3f(0.f,1.f,0.f)));
      // std::cout << d->getMatrix() << std::endl;
      d->setupDefaultGeometry();
      return d;
    }

    osgManipulator::Dragger* createDragger(const DraggerType& type)
    {
      osgManipulator::Dragger* dragger = 0;
      switch (type) {
        case TabPlaneDragger:
          dragger = newDragger<TabPlaneDraggerXY>();
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

    NodeDragger::NodeDragger(float scaling) :
      scaling_ (scaling), rootChild_ (NULL)
    {}

    bool NodeDragger::empty()
    {
      return (rootChild_ == NULL || scene_.get()==NULL || selection_.get()==NULL);
    }

    void NodeDragger::addDragger(const DraggerType& type, bool fixedSizeInScreen)
    {
      removeDragger();

      dragger_ = createDragger(type);

      if ( fixedSizeInScreen )
      {
        DraggerContainer* draggerContainer = new DraggerContainer;
        draggerContainer->setDragger( dragger_ );
        rootChild_ = draggerContainer;
      }
      else
        rootChild_ = dragger_;
      scene_->addChild(rootChild_);

      float scale = scene_->getBound().radius() * scaling_;
      dragger_->setMatrix(osg::Matrix::scale(scale, scale, scale) *
                          osg::Matrix::translate(scene_->getBound().center()));

      dragger_->addTransformUpdating(selection_);

      // we want the dragger to handle it's own events automatically
      dragger_->setHandleEvents(true);

      // if we don't set an activation key or mod mask then any mouse click on
      // the dragger will activate it, however if do define either of ActivationModKeyMask or
      // and ActivationKeyEvent then you'll have to press either than mod key or the specified key to
      // be able to activate the dragger when you mouse click on it.  Please note the follow allows
      // activation if either the ctrl key or the 'a' key is pressed and held down.
      dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
      //dragger->setActivationKeyEvent('a');
    }

    void NodeDragger::setSceneAndTransform(osg::Group* scene, osg::MatrixTransform* selection)
    {
      // scene->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
      if (!empty()) removeDragger();
      scene_ = scene;
      selection_ = selection;
    }

    void NodeDragger::removeDragger ()
    {
      if (!empty()) {
        scene_->removeChild(rootChild_);
        rootChild_ = NULL;
      }
    }
  } /* namespace nodeManipulation */
} /* namespace graphics */
