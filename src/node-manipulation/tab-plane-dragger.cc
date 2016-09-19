/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/
//osgManipulator - Copyright (C) 2007 Fugro-Jason B.V.

#ifndef SCENEVIEWER_NODE_MANIPULATOR_TAB_PLANE_DRAGGER_HH
#define SCENEVIEWER_NODE_MANIPULATOR_TAB_PLANE_DRAGGER_HH

#include <osgManipulator/TranslatePlaneDragger>
#include <osgManipulator/Scale2DDragger>
#include <osgManipulator/Scale1DDragger>
#include <osgManipulator/Dragger>

#include <osgManipulator/AntiSquish>

#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Quat>
#include <osg/PolygonMode>
#include <osg/CullFace>
#include <osg/AutoTransform>

#include <assert.h>

namespace graphics {
  namespace nodeManipulation {
    std::ostream& operator<< (std::ostream& os, const osg::Matrix& m) {
      for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j)
          os << m(i,j) << ", ";
      }
      os << "\n";
      return os;
    }

    /**
     * Tab plane dragger consists of a plane with tabs on it's corners and edges
     * for scaling. And the plane is used as a 2D translate dragger.
     */
    template <int Axis1, int Axis2>
      class TabPlaneDraggerTpl : public ::osgManipulator::CompositeDragger
    {
      public:
        typedef ::osgManipulator::Translate2DDragger Translate2DDragger;
        typedef ::osgManipulator::Scale2DDragger     Scale2DDragger;
        typedef ::osgManipulator::Scale1DDragger     Scale1DDragger;

        TabPlaneDraggerTpl(float handleScaleFactor=20.0f);

        META_OSGMANIPULATOR_Object(osgManipulator,TabPlaneDraggerTpl)

          virtual bool handle(const ::osgManipulator::PointerInfo& pi, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);

        /** Setup default geometry for dragger. */
        void setupDefaultGeometry(bool twoSidedHandle = true);

        void setPlaneColor(const osg::Vec4& color) { _translateDragger->setColor(color); }

      protected:

        virtual ~TabPlaneDraggerTpl();

        osg::ref_ptr< Translate2DDragger >   _translateDragger;
        osg::ref_ptr< Scale2DDragger >       _cornerScaleDragger;
        osg::ref_ptr< Scale1DDragger >       _horzEdgeScaleDragger;
        osg::ref_ptr< Scale1DDragger >       _vertEdgeScaleDragger;

        float                                   _handleScaleFactor;
    };

    // -------------------------- Implementation ----------------------------------

    namespace {
      template <int Axis1, int Axis2> osg::Vec3d vector(const double& v1, const double& v2)
      {
        ::osg::Vec3d v (0,0,0); v[Axis1] = v1; v[Axis2] = v2;
        return v;
      }
      template <int Axis1, int Axis2> osg::Vec3d vector(const osg::Vec2d& v)
      {
        return vector<Axis1, Axis2> (v[0],v[1]);
      }
      template <int Axis1, int Axis2> osg::Vec3d normal()
      {
        ::osg::Vec3d v (1,1,1); v[Axis1] = 0; v[Axis2] = 0;
        return v;
      }

      template <int Axis1, int Axis2>
        osg::Node* createHandleNode(typename TabPlaneDraggerTpl<Axis1,Axis2>::Scale2DDragger* cornerScaleDragger, float handleScaleFactor, bool twosided)
        {
          osg::Vec3Array* vertices = new osg::Vec3Array(4);
          (*vertices)[0] = vector<Axis1,Axis2>(cornerScaleDragger->getTopLeftHandlePosition()) * handleScaleFactor;
          (*vertices)[1] = vector<Axis1,Axis2>(cornerScaleDragger->getBottomLeftHandlePosition()) * handleScaleFactor;
          (*vertices)[2] = vector<Axis1,Axis2>(cornerScaleDragger->getBottomRightHandlePosition()) * handleScaleFactor;
          (*vertices)[3] = vector<Axis1,Axis2>(cornerScaleDragger->getTopRightHandlePosition()) * handleScaleFactor;

          // std::cout << (*vertices)[0][0] << " " << (*vertices)[0][1] << " " << (*vertices)[0][2]<< std::endl;

          osg::Geometry* geometry = new osg::Geometry();
          geometry->setVertexArray(vertices);
          geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,vertices->size()));

          osg::Vec3Array* normals = new osg::Vec3Array;
          normals->push_back(normal<Axis1,Axis2>());
          geometry->setNormalArray(normals, osg::Array::BIND_OVERALL);

          osg::Geode* geode = new osg::Geode;
          geode->setName("Dragger Handle");
          geode->addDrawable(geometry);

          if (!twosided)
          {
            osg::CullFace* cullface = new osg::CullFace;
            cullface->setMode(osg::CullFace::FRONT);
            geode->getOrCreateStateSet()->setAttribute(cullface, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            geode->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
          }

          geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

          return geode;
        }

      osg::Group* createHandleScene(const osg::Vec3& pos, osg::Node* handleNode, float handleScaleFactor)
      {
        osg::AutoTransform *at = new osg::AutoTransform;
        at->setPosition(pos);
        at->setPivotPoint(pos * handleScaleFactor);
        at->setAutoScaleToScreen(true);
        at->addChild(handleNode);

        ::osgManipulator::AntiSquish* as = new ::osgManipulator::AntiSquish;
        as->setPivot(pos);
        as->addChild(at);

        return as;
      }

      template <int Axis1, int Axis2>
        void createCornerScaleDraggerGeometry(typename TabPlaneDraggerTpl<Axis1,Axis2>::Scale2DDragger* cornerScaleDragger, osg::Node* handleNode, float handleScaleFactor)
        {
          ::osg::Matrix m; m.makeIdentity();
          m(0,0) = m(1,1) = m(2,2) = 0;
          m(Axis1,0) =  1; m(Axis2,2) = -1; m(3-Axis1-Axis2,1) = 1;
          std::cout << m << std::endl;
          osg::Quat q = m.getRotate();
          std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;

          // std::cout << cornerScaleDragger->getBottomLeftHandlePosition()[0] << " " ;
          // std::cout << cornerScaleDragger->getBottomLeftHandlePosition()[1] << std::endl;

          // Create a top left box.
          {
            osg::Group* handleScene = createHandleScene(
                vector<0, 2>(cornerScaleDragger->getTopLeftHandlePosition()),
                handleNode, handleScaleFactor);
            dynamic_cast<osg::AutoTransform*>(handleScene->getChild(0))->asAutoTransform()->setRotation(q);
            cornerScaleDragger->addChild(handleScene);
            cornerScaleDragger->setTopLeftHandleNode(*handleScene);
          }

          // Create a bottom left box.
          {
            osg::Group* handleScene = createHandleScene(
                vector<0, 2>(cornerScaleDragger->getBottomLeftHandlePosition()),
                handleNode, handleScaleFactor);
            dynamic_cast<osg::AutoTransform*>(handleScene->getChild(0))->asAutoTransform()->setRotation(q);
            cornerScaleDragger->addChild(handleScene);
            cornerScaleDragger->setBottomLeftHandleNode(*handleScene);
          }

          // Create a bottom right box.
          {
            osg::Group* handleScene = createHandleScene(
                vector<0, 2>(cornerScaleDragger->getBottomRightHandlePosition()),
                handleNode, handleScaleFactor);
            dynamic_cast<osg::AutoTransform*>(handleScene->getChild(0))->asAutoTransform()->setRotation(q);
            cornerScaleDragger->addChild(handleScene);
            cornerScaleDragger->setBottomRightHandleNode(*handleScene);
          }

          // Create a top right box.
          {
            osg::Group* handleScene = createHandleScene(
                vector<0, 2>(cornerScaleDragger->getTopRightHandlePosition()),
                handleNode, handleScaleFactor);
            dynamic_cast<osg::AutoTransform*>(handleScene->getChild(0))->asAutoTransform()->setRotation(q);
            cornerScaleDragger->addChild(handleScene);
            cornerScaleDragger->setTopRightHandleNode(*handleScene);
          }
        }

      template <int Axis1, int Axis2>
        void createEdgeScaleDraggerGeometry(typename TabPlaneDraggerTpl<Axis1,Axis2>::Scale1DDragger* horzEdgeScaleDragger, ::osgManipulator::Scale1DDragger* vertEdgeScaleDragger,
            osg::Node* handleNode, float handleScaleFactor)
        {
          // Create a left box.
          {
            osg::Node* handleScene = createHandleScene(vector<Axis1, Axis2>(horzEdgeScaleDragger->getLeftHandlePosition(),0.0),
                handleNode, handleScaleFactor);
            horzEdgeScaleDragger->addChild(handleScene);
            horzEdgeScaleDragger->setLeftHandleNode(*handleScene);
          }

          // Create a right box.
          {
            osg::Node* handleScene = createHandleScene(vector<Axis1, Axis2>(horzEdgeScaleDragger->getRightHandlePosition(),0.0),
                handleNode, handleScaleFactor);
            horzEdgeScaleDragger->addChild(handleScene);
            horzEdgeScaleDragger->setRightHandleNode(*handleScene);
          }

          // Create a top box.
          {
            osg::Node* handleScene = createHandleScene(vector<Axis1, Axis2>(vertEdgeScaleDragger->getLeftHandlePosition(),0.0),
                handleNode, handleScaleFactor);
            vertEdgeScaleDragger->addChild(handleScene);
            vertEdgeScaleDragger->setLeftHandleNode(*handleScene);
          }

          // Create a bottom box.
          {
            // osg::Node* handleScene = createHandleScene(vector<Axis1, Axis2>(0.0,vertEdgeScaleDragger->getRightHandlePosition()),
            osg::Node* handleScene = createHandleScene(vector<Axis1, Axis2>(vertEdgeScaleDragger->getRightHandlePosition(),0.0),
                handleNode, handleScaleFactor);
            vertEdgeScaleDragger->addChild(handleScene);
            vertEdgeScaleDragger->setRightHandleNode(*handleScene);
          }

          // osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(1.0f, 0.0f, 0.0f));
          osg::Quat rotation; rotation.makeRotate(vector<Axis1,Axis2>(0.0,1.0), vector<Axis1,Axis2>(1.0,0.0));
          vertEdgeScaleDragger->setMatrix(osg::Matrix(rotation));
        }

      template <int Axis1, int Axis2>
        void createTranslateDraggerGeometry(typename TabPlaneDraggerTpl<Axis1,Axis2>::Scale2DDragger* cornerScaleDragger, ::osgManipulator::Translate2DDragger* translateDragger)
        {
          // Create a polygon.
          {
            osg::Geode* geode = new osg::Geode;
            osg::Geometry* geometry = new osg::Geometry();

            osg::Vec3Array* vertices = new osg::Vec3Array(4);
            (*vertices)[0] = vector<Axis1,Axis2>(cornerScaleDragger->getTopLeftHandlePosition());
            (*vertices)[1] = vector<Axis1,Axis2>(cornerScaleDragger->getBottomLeftHandlePosition());
            (*vertices)[2] = vector<Axis1,Axis2>(cornerScaleDragger->getBottomRightHandlePosition());
            (*vertices)[3] = vector<Axis1,Axis2>(cornerScaleDragger->getTopRightHandlePosition());

            geometry->setVertexArray(vertices);
            geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,vertices->size()));

            osg::Vec3Array* normals = new osg::Vec3Array;
            normals->push_back(normal<Axis1,Axis2>());
            geometry->setNormalArray(normals, osg::Array::BIND_OVERALL);

            geode->addDrawable(geometry);

            osg::PolygonMode* polymode = new osg::PolygonMode;
            polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
            geode->getOrCreateStateSet()->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

            geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

            translateDragger->addChild(geode);
          }

        }
    }

    template <int Axis1, int Axis2>
      TabPlaneDraggerTpl<Axis1,Axis2>::TabPlaneDraggerTpl( float handleScaleFactor )
      :_handleScaleFactor( handleScaleFactor )
      {
        ::osg::Matrix m; m.makeIdentity();
        m(0,0) = m(1,1) = m(2,2) = 0;
        m(Axis1,0) = 1; m(Axis2,2) = -1; m(3-Axis1-Axis2,1) = 1;
        std::cout << m << std::endl;
        setMatrix(m);

        _cornerScaleDragger = new Scale2DDragger(Scale2DDragger::SCALE_WITH_OPPOSITE_HANDLE_AS_PIVOT);
        _cornerScaleDragger->setMatrix(m);
        addChild(_cornerScaleDragger.get());
        addDragger(_cornerScaleDragger.get());

        _horzEdgeScaleDragger = new Scale1DDragger(::osgManipulator::Scale1DDragger::SCALE_WITH_OPPOSITE_HANDLE_AS_PIVOT);
        addChild(_horzEdgeScaleDragger.get());
        addDragger(_horzEdgeScaleDragger.get());

        _vertEdgeScaleDragger = new Scale1DDragger(::osgManipulator::Scale1DDragger::SCALE_WITH_OPPOSITE_HANDLE_AS_PIVOT);
        addChild(_vertEdgeScaleDragger.get());
        addDragger(_vertEdgeScaleDragger.get());

        _translateDragger = new Translate2DDragger(::osg::Plane(normal<Axis1,Axis2>(), 0));
        _translateDragger->setColor(osg::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
        addChild(_translateDragger.get());
        addDragger(_translateDragger.get());

        setParentDragger(getParentDragger());
      }

    template <int Axis1, int Axis2>
      TabPlaneDraggerTpl<Axis1,Axis2>::~TabPlaneDraggerTpl()
      {
      }

    template <int Axis1, int Axis2>
      bool TabPlaneDraggerTpl<Axis1,Axis2>::handle(const ::osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
      {
        if (ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) return false;

        // Check if the dragger node is in the nodepath.
        if (!pointer.contains(this)) return false;

        // Since the translate plane and the handleNode lie on the same plane the hit could've been on either one. But we
        // need to handle the scaling draggers before the translation. Check if the node path has the scaling nodes else
        // check for the scaling nodes in next hit.
        if (_cornerScaleDragger->handle(pointer, ea, aa))
          return true;
        if (_horzEdgeScaleDragger->handle(pointer, ea, aa))
          return true;
        if (_vertEdgeScaleDragger->handle(pointer, ea, aa))
          return true;

        ::osgManipulator::PointerInfo nextPointer(pointer);
        nextPointer.next();

        while (!nextPointer.completed())
        {
          if (_cornerScaleDragger->handle(nextPointer, ea, aa))
            return true;
          if (_horzEdgeScaleDragger->handle(nextPointer, ea, aa))
            return true;
          if (_vertEdgeScaleDragger->handle(nextPointer, ea, aa))
            return true;

          nextPointer.next();
        }

        if (_translateDragger->handle(pointer, ea, aa))
          return true;

        return false;
      }

    template <int Axis1, int Axis2>
      void TabPlaneDraggerTpl<Axis1,Axis2>::setupDefaultGeometry(bool twoSidedHandle)
      {
        osg::ref_ptr<osg::Node> handleNode = createHandleNode<Axis1,Axis2>(_cornerScaleDragger.get(), _handleScaleFactor, twoSidedHandle);

        createCornerScaleDraggerGeometry<Axis1,Axis2>(_cornerScaleDragger.get(), handleNode.get(), _handleScaleFactor);
        createEdgeScaleDraggerGeometry<Axis1,Axis2>(_horzEdgeScaleDragger.get(),_vertEdgeScaleDragger.get(),handleNode.get(),_handleScaleFactor);
        createTranslateDraggerGeometry<Axis1,Axis2>(_cornerScaleDragger.get(), _translateDragger.get());
      }
  }
}

#endif // SCENEVIEWER_NODE_MANIPULATOR_TAB_PLANE_DRAGGER_HH
