#ifndef SCENEVIEWER_FPSMANIPULATOR_H
#define SCENEVIEWER_FPSMANIPULATOR_H
//
//  KeyboardManipulator
//  gepetto-viewer
//
//  Alternative CameraManipulator for OSG, use keyboard and mouse
//  KeyBinding are inspired by the classic system in games
//
//  Created by Pierre Fernbach in january 2016
//


#include <osgGA/FirstPersonManipulator>
#include <osgViewer/Viewer>
#include <osg/Camera>
#include <osgViewer/GraphicsWindow>
#include <X11/Xlib.h>




namespace osgGA {

  const double startSpeed_ = 2.;
  /** FirstPersonManipulator is base class for camera control based on position
      and orientation of camera, like walk, drive, and flight manipulators. */
  class OSGGA_EXPORT KeyboardManipulator : public FirstPersonManipulator
  {
          typedef FirstPersonManipulator inherited;

  public:
    KeyboardManipulator( int flags = DEFAULT_SETTINGS );
    KeyboardManipulator( const KeyboardManipulator& fpm, const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY );
    /// Constructor with reference to the graphic window, needed for hidding mouse cursor
    KeyboardManipulator(osgViewer::GraphicsWindow* window, int flags = DEFAULT_SETTINGS );

    META_Object( osgGA, KeyboardManipulator );

  protected :

    virtual bool handleKeyDown( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    virtual bool handleKeyUp( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    virtual bool handleFrame( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    virtual bool handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    virtual bool handleMouseRelease( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
  //  virtual bool handleMouseWheel( const GUIEventAdapter& ea, GUIActionAdapter& us );
    virtual bool performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy );

    virtual void getUsage(osg::ApplicationUsage &usage) const;
    bool initKeyboard();
  private :

    double speed_;
    osg::Vec3d speedT_, speedR_;
    osg::Quat rotateRoll_;
    osg::Vec3d localUp_;
    int keyLayout_;

   //osg::Camera* camera_;
    osgViewer::GraphicsWindow* gWindow_;
   // Display *display_;
    int keycode_;
    bool rightClic_;
    bool ctrl_;
    bool shift_;
    bool noRoll_;


  };// end class

  enum keyLayout{
    LAYOUT_unknown,LAYOUT_azerty,LAYOUT_qwerty
  };
}//namespace osgGA
#endif // FPSMANIPULATOR_H
