#ifndef NGLSCENE_H_
#define NGLSCENE_H_
#include <ngl/Vec3.h>
#include <ngl/Mat4.h>
#include "WindowParams.h"
// this must be included after NGL includes else we get a clash with gl libs
#include <QOpenGLWindow>

#include <ngl/VAOPrimitives.h>
#include <ngl/obj.h>
#include <ngl/ShaderLib.h>
#include <ngl/Transformation.h>
#include <ngl/NGLStream.h>
#include <iostream>
#include <ngl/BBox.h>
#include <Sphere.h>
#include <memory>
#include <QEvent>
#include <QResizeEvent>
#include <QOpenGLWidget>




//----------------------------------------------------------------------------------------------------------------------
/// @file NGLScene.h
/// @brief this class inherits from the Qt OpenGLWindow and allows us to use NGL to draw OpenGL
/// @author Jonathan Macey
/// @version 1.0
/// @date 10/9/13
/// Revision History :
/// This is an initial version used for the new NGL6 / Qt 5 demos
/// @class NGLScene
/// @brief our main glwindow widget for NGL applications all drawing elements are
/// put in this file
//----------------------------------------------------------------------------------------------------------------------


class NGLScene : public QOpenGLWidget
{
  Q_OBJECT
  public:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief ctor for our NGL drawing class
    /// @param [in] parent the parent window to the class
    //----------------------------------------------------------------------------------------------------------------------
    NGLScene(QWidget *_parent);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief dtor must close down ngl and release OpenGL resources
    //----------------------------------------------------------------------------------------------------------------------
    ~NGLScene() override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the initialize class is called once when the window is created and we have a valid GL context
    /// use this to setup any default GL stuff
    //----------------------------------------------------------------------------------------------------------------------
    void initializeGL() override;  
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we want to draw the scene
    //----------------------------------------------------------------------------------------------------------------------
    void paintGL() override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we resize the window
    //----------------------------------------------------------------------------------------------------------------------
    void resizeGL(int _w, int _h) override;

    void timerEvent(QTimerEvent *_event ) override;

    public slots:
      // set position
      void setXPosition(double _posX);
      void setYPosition(double _posY);
      void setZPosition(double _posZ);

      // set velocity
      void setXVelocity(double _velX);
      void setYVelocity(double _velY);
      void setZVelocity(double _velZ);

      // Physics
      void setMass(double _mass);
      void setRadius(double _radius);
      void setGravity(double _gravity);
      void setEnergyLoss(double _energyLoss);
      void setFriction(double _meu);

      // colour
      void setColR(int _colR);
      void setColG(int _colG);
      void setColB(int _colB);

      // sim controls
      void setMinRadius(double _minRadius);
      void setMaxRadius(double _maxRadius);

      void setMinMass(double _minMass);
      void setMaxMass(double _maxMass);

      void setNumSpheres(int _numSpheres);

      // bbox
      void setBBoxSize(double _x);

      // Scene controls
      void addSphere();
      void resetScene();
      void startSim();
      void randomise();

private:
   //----------------------------------------------------------------------------------------------------------------------
    /// @brief Qt Event called when a key is pressed
    /// @param [in] _event the Qt event to query for size etc
    //----------------------------------------------------------------------------------------------------------------------
    void keyPressEvent(QKeyEvent *_event) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called every time a mouse is moved
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mouseMoveEvent (QMouseEvent * _event ) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse button is pressed
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mousePressEvent ( QMouseEvent *_event) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse button is released
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mouseReleaseEvent ( QMouseEvent *_event ) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse wheel is moved
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void wheelEvent( QWheelEvent *_event) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the
    /// @param _event the Qt Event structure
    WinParams m_win;

    // Timer to store number of milliseconds since the last update
    int physics_timer;

    ngl::Mat4 m_mouseGlobalTX;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the model position for mouse movement
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Vec3 m_modelPos;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the view matrix for camera
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Mat4 m_view;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the projection matrix for camera
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Mat4 m_project;

    ngl::Mat4 m_globalTransformMatrix;
    ngl::Mat4 m_bodyTransform;

    // arrays to store spheres
    std::vector<Sphere> m_spheres;
    std::vector<Sphere> m_randomSpheres;
    
    // bounding box
    std::unique_ptr<ngl::BBox> m_bbox;

    // Physics
    void bboxCollision();
    ngl::Vec3 resolveBoxOverlap(ngl::Vec3 _Planenormal, ngl::Vec3 _pos, ngl::Real _rad);
    bool detectSphereCollision(ngl::Vec3 _pos1, ngl::Vec3 _pos2, float _rad1, float _rad2);
    void checkSphereCollisions();
    std::pair<ngl::Vec3, ngl::Vec3> resolveSphereCollision(ngl::Vec3 _pos1, ngl::Vec3 _pos2, ngl::Vec3 _vel1, ngl::Vec3 _vel2, float _m1, float _m2);
    ngl::Vec3 resolveSphereOverlap(ngl::Vec3 _p1, ngl::Vec3 _p2, ngl::Real _r1, ngl::Real _r2);

    // Shaders
    void loadMatricesToShader();
    void loadMatricesToColourShader();

    void updateSim();

    float GenerateRandomFloat(float _min, float _max);

    // Physical quantities
    ngl::Vec3 m_gravity = {0.0f, -0.05f, 0.0f};
    ngl::Real m_energyLoss = 0.2f; // percentage of energy loss
    ngl::Real m_meu = 0.03f; // kinetic energy

    // Sphere properties
    double m_minRadius = 0.5;
    double m_maxRadius = 1.0;

    double m_minMass = 0.5;
    double m_maxMass = 1.0;

    int m_numSpheres = 10;
    int m_totalNumSpheres = 0;

    // simulation start / pause check
    bool m_runSim;
};

#endif
