#include <QMouseEvent>
#include <QGuiApplication>
#include "NGLScene.h"
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Mat4.h>
#include <ngl/Util.h>
#include <ngl/NGLStream.h>
#include <ngl/Transformation.h>
#include <iostream>
#include <algorithm>
#include <ngl/Random.h>
#include "Sphere.h"
#include <QApplication>

NGLScene::NGLScene(QWidget *_parent): QOpenGLWidget(_parent)
{
  m_runSim = false;
  setFocus();
  this->resize(_parent->size());
}

NGLScene::~NGLScene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

// UI functions
void NGLScene::addSphere()
{
  Sphere newSphere = Sphere({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 1.0f, 1.0f, {1.0f, 1.0f, 1.0f, 1.0f});
  m_spheres.push_back(newSphere);
  m_totalNumSpheres++;
  update();
}

void NGLScene::startSim()
{
  if(m_runSim) {m_runSim = false;}
  else {m_runSim = true;}
}

void NGLScene::randomise()
{
  m_randomSpheres.resize(m_numSpheres);
  // iterate through m_spheresArray by address
  for(auto &i : m_randomSpheres)
  {
    float boxSize = m_bbox->height();
    
    ngl::Real radius = GenerateRandomFloat(m_minRadius, m_maxRadius);
    ngl::Vec3 pos = ngl::Random::getRandomPoint(boxSize/2.3 - radius, boxSize/2.3 - radius, boxSize/2.3 - radius);
    ngl::Vec3 vel = ngl::Random::getRandomVec3();
    ngl::Real mass = GenerateRandomFloat(m_minMass, m_maxMass);
    ngl::Vec4 colour = ngl::Random::getRandomColour4();
    
    i = Sphere(pos, vel, radius, mass, colour);
    i.updatePosAndVel(); 
    m_spheres.push_back(i);
    m_totalNumSpheres++;
  }
  update();
}

void NGLScene::resetScene()
{
  m_totalNumSpheres = 0;
  m_spheres.clear();
  update();
}

void NGLScene::setColR(int _colR)
{
  if(m_totalNumSpheres != 0)
  {
    float red = _colR / 255.0f;
    ngl::Vec4 colour = {red, m_spheres[m_totalNumSpheres - 1].getColour().m_y, m_spheres[m_totalNumSpheres - 1].getColour().m_z, 1.0f};
    m_spheres[m_totalNumSpheres - 1].setColour(colour);
    update();
  }
}

void NGLScene::setColG(int _colG)
{
  if(m_totalNumSpheres != 0)
  {
    float green = _colG / 255.0f;
    ngl::Vec4 colour = {m_spheres[m_totalNumSpheres - 1].getColour().m_x, green, m_spheres[m_totalNumSpheres - 1].getColour().m_z, 1.0f};
    m_spheres[m_totalNumSpheres - 1].setColour(colour);
    update();
  }
}

void NGLScene::setColB(int _colB)
{
  if(m_totalNumSpheres != 0)
  {
    float blue = _colB / 255.0f;
    ngl::Vec4 colour = {m_spheres[m_totalNumSpheres - 1].getColour().m_x, m_spheres[m_totalNumSpheres - 1].getColour().m_y, blue, 1.0f};
    m_spheres[m_totalNumSpheres - 1].setColour(colour);
    update();
  }
}

void NGLScene::setXPosition(double _x)
{
  if(m_totalNumSpheres != 0)
  {
    m_spheres[m_totalNumSpheres - 1].setPos(ngl::Vec3(_x, m_spheres[m_totalNumSpheres - 1].getPos().m_y, m_spheres[0].getPos().m_z));
    m_spheres[m_totalNumSpheres - 1].updatePos();
    update();
  }
}

void NGLScene::setYPosition(double _y)
{
  if(m_totalNumSpheres != 0)
  {
    m_spheres[m_totalNumSpheres - 1].setPos(ngl::Vec3( m_spheres[m_totalNumSpheres - 1].getPos().m_x, _y, m_spheres[0].getPos().m_z));
    m_spheres[m_totalNumSpheres - 1].updatePos();
    update();
  }
}

void NGLScene::setZPosition(double _z)
{
  if(m_totalNumSpheres != 0)
  {
    m_spheres[m_totalNumSpheres - 1].setPos(ngl::Vec3( m_spheres[m_totalNumSpheres - 1].getPos().m_x, m_spheres[0].getPos().m_y, _z));
    m_spheres[m_totalNumSpheres - 1].updatePos();
    update();
  }
}

void NGLScene::setXVelocity(double _x)
{
  if(m_totalNumSpheres != 0)
  {
    m_spheres[m_totalNumSpheres - 1].setVel(ngl::Vec3(_x, m_spheres[m_totalNumSpheres - 1].getVel().m_y, m_spheres[0].getVel().m_z));
  }
}

void NGLScene::setYVelocity(double _y)
{
  if(m_totalNumSpheres != 0)
  {
    m_spheres[m_totalNumSpheres - 1].setVel(ngl::Vec3( m_spheres[m_totalNumSpheres - 1].getVel().m_x, _y, m_spheres[0].getVel().m_z));
  }
}

void NGLScene::setZVelocity(double _z)
{
  if(m_totalNumSpheres != 0)
  {
    m_spheres[m_totalNumSpheres - 1].setVel(ngl::Vec3( m_spheres[m_totalNumSpheres - 1].getVel().m_x, m_spheres[0].getVel().m_y, _z));
  }
}

void NGLScene::setFriction(double _f){m_meu = _f / 10;}
void NGLScene::setGravity(double _g){m_gravity.m_y = _g / 196.2;}
void NGLScene::setEnergyLoss(double _e){m_energyLoss = _e;}
void NGLScene::setMinRadius(double _r){m_minRadius = _r;}
void NGLScene::setMaxRadius(double _r){m_maxRadius = _r;}
void NGLScene::setMinMass(double _m){m_minMass = _m;}
void NGLScene::setMaxMass(double _m){m_maxMass = _m;}
void NGLScene::setNumSpheres(int _n){m_numSpheres = _n;}


void NGLScene::setRadius(double _r)
{
  int numSpheres = int(m_spheres.size());
  if(numSpheres > 0)
  {
    m_spheres[numSpheres - 1].setRadius(_r);
  }
}

void NGLScene::setMass(double _m)
{
  int numSpheres = int(m_spheres.size());
  if(numSpheres > 0)
  {
    m_spheres[numSpheres - 1].setMass(_m);
  }
}

void NGLScene::setBBoxSize(double _x)
{
  m_bbox->setExtents(-_x/2, _x/2, -_x/2, _x/2, -_x/2, _x/2);
  m_bbox->recalculate();
  update();
}


// NGL functions

void NGLScene::resizeGL(int _w , int _h)
{
  m_project = ngl::perspective(45.0f, static_cast<float>(_w) / _h, 0.05f, 350.0f);
  m_win.width  = static_cast<int>( _w * devicePixelRatio() );
  m_win.height = static_cast<int>( _h * devicePixelRatio() );
}

float NGLScene::GenerateRandomFloat(float _min, float _max)
{
  float random = static_cast<float>(rand()) / RAND_MAX;  // Scale to [0, 1]
  return _min + (random * (_max - _min));  // Scale to [min, max]
}

void NGLScene::initializeGL()
{
  // we must call that first before any other GL commands to load and link the
  // gl commands from the lib, if that is not done program will crash
  ngl::NGLInit::initialize();
  glClearColor(0.7f, 0.7f, 0.7f, 1.0f);			   // Grey Background
  // enable depth testing for   ngl::NGLInit::initialize();

  glEnable(GL_DEPTH_TEST);
  // enable multisampling for smoother drawing
  glEnable(GL_MULTISAMPLE);
  
  ngl::ShaderLib::loadShader("ColourShader","shaders/ColourVertex.glsl",
  "shaders/ColourFragment.glsl");

  // create sphere instance
  ngl::VAOPrimitives::createSphere("sphere", 1.0f, 30);



  // create a default sphere for the default scene
  ngl::Vec3 pos = {0.0f, 0.0f, 0.0f};
  ngl::Vec3 vel = {0.0f, 0.0f, 0.0f};
  ngl::Vec4 col = {1.0, 1.0f, 1.0f, 1.0f};

  Sphere sphere1(pos, vel, 1.0f, 1.0f, col);
  m_spheres.push_back(sphere1);
  m_totalNumSpheres++;


  // We now create our view matrix for a static camera
  ngl::Vec3 from{0.0f, 2.0f, 20.0f};
  ngl::Vec3 to{0.0f, 0.0f, 0.0f};
  ngl::Vec3 up{0.0f, 3.0f, 0.0f};

  // now load to our new camera
  m_view = ngl::lookAt(from, to, up);
  m_project = ngl::perspective(45.0f, 720.0f / 576.0f, 0.5f, 150.0f);


  ngl::ShaderLib::use("nglDiffuseShader");

  ngl::ShaderLib::setUniform("Colour", 0.0f, 1.0f, 0.0f, 1.0f);
  ngl::ShaderLib::setUniform("lightPos", 1.0f, 1.0f, 1.0f);
  ngl::ShaderLib::setUniform("lightDiffuse", 1.0f, 1.0f, 1.0f, 1.0f);

  ngl::ShaderLib::use("nglColourShader");
  ngl::ShaderLib::setUniform("Colour", 1.0f, 1.0f, 1.0f, 1.0f);

  glEnable(GL_DEPTH_TEST); // for removal of hidden surfaces
  
  // create our Bounding Box, needs to be done once we have a gl context as we create VAO for drawing
  ngl::Vec3 centre = {0.0f, 0.0f, 0.0f};
  m_bbox = std::make_unique<ngl::BBox>(centre, 10, 10, 10);

  physics_timer = startTimer(20);
}

void NGLScene::loadMatricesToShader()
{
  ngl::ShaderLib::use("nglDiffuseShader");
  ngl::Mat4 MV;
  ngl::Mat4 MVP; 
  ngl::Mat3 normalMatrix;
  MV = m_view * m_mouseGlobalTX;
  MVP = m_project * MV;
  normalMatrix = MV;
  normalMatrix.inverse().transpose();
  ngl::ShaderLib::setUniform("MVP", MVP);
  ngl::ShaderLib::setUniform("normalMatrix", normalMatrix);
}

void NGLScene::loadMatricesToColourShader()
{
  ngl::ShaderLib::use("nglColourShader");
  ngl::Mat4 MVP;
  MVP = m_project * m_view * m_mouseGlobalTX;
  ngl::ShaderLib::setUniform("MVP", MVP);
}


// Sphere-sphere collision resolution algorithm by Kent
// Website: Studio Freya, Simple Sphere-Sphere Collision Detection and Collision Response, [online], Accessed: 2023
// Available from: https://studiofreya.com/3d-math-and-physics/simple-sphere-sphere-collision-detection-and-collision-response/
std::pair<ngl::Vec3, ngl::Vec3> NGLScene::resolveSphereCollision(ngl::Vec3 _pos1, ngl::Vec3 _pos2, ngl::Vec3 _vel1, ngl::Vec3 _vel2, float _mass1, float _mass2)
{
  // Find the vector which will serve as a basis vector (x-axis), 
  // in an arbitrary direction. It has to be normalized to get realistic results.
  ngl::Vec3 x = _pos1 - _pos2;
  x.normalize();
  

  // Calculate the x-direction velocity vector and the perpendicular y-vector.
  ngl::Vec3 v1 = _vel1;
  ngl::Real x1 = x.dot(v1);

  ngl::Vec3 v1x = x *x1;
  ngl::Vec3 v1y = v1 - v1x;
  ngl::Real m1 = _mass1;

  // Same procedure for the other sphere.
  x = x*-1;
  ngl::Vec3 v2 = _vel2;
  ngl::Real x2 = x.dot(v2);

  ngl::Vec3 v2x = x *x2;
  ngl::Vec3 v2y = v2 - v2x;
  ngl::Real m2 = _mass2;

  // Apply Newton's laws to obtain a formula for speed after collision.
  ngl::Vec3 finalVel1 = (v1x*((m1-m2)/(m1+m2)))+(v2x*((2*m2)/(m1+m2)))+v1y;
  ngl::Vec3 finalVel2 = (v1x*((2*m2)/(m1+m2)))+(v2x*((m2-m1)/(m1+m2)))+v2y;

  return std::make_pair(finalVel1, finalVel2);
}
// End of algorithm by Kent.


void NGLScene::checkSphereCollisions()
{
  bool collision;

  for (Sphere &ToCheck : m_spheres)
  {
    for (Sphere &Current : m_spheres)
    {
      // don't check against self
      if (ToCheck.getPos() == Current.getPos())
        continue;
      else
      {
        collision = detectSphereCollision(Current.getPos(), ToCheck.getPos(), Current.getRadius(), ToCheck.getRadius());        
        if(collision)
        {
          ngl::Vec3 correction = resolveSphereOverlap(Current.getPos(), ToCheck.getPos(), Current.getRadius(), ToCheck.getRadius());
          
          Current.setPos(Current.getPos() += correction);
          ToCheck.setPos(ToCheck.getPos() -= correction);

          std::pair<ngl::Vec3, ngl::Vec3> finalVelocities = resolveSphereCollision(Current.getPos(), ToCheck.getPos(),
                                                                                 Current.getVel(), ToCheck.getVel(), 
                                                                               Current.getMass(), ToCheck.getMass());
          
          Current.setVel(finalVelocities.first * (1 - m_energyLoss));
          ToCheck.setVel(finalVelocities.second * (1 - m_energyLoss));
          
          Current.updatePos();
          ToCheck.updatePos();
        }
      }
    }
  }
}

bool NGLScene::detectSphereCollision(ngl::Vec3 _pos1, ngl::Vec3 _pos2, float _rad1, float _rad2)
{
  // Calculate distance squared between the centres of the spheres
  ngl::Vec3 vector = _pos1 - _pos2;
  float dist = vector.lengthSquared();  

  // Calculate the square of the sum of both the radii
  float minDist = (_rad1 + _rad2) * (_rad1 + _rad2); 

  // Check if the spheres are colliding
  if (dist <= minDist) 
  {
    return true;
  }
  else return false;
}

ngl::Vec3 NGLScene::resolveSphereOverlap(ngl::Vec3 _p1, ngl::Vec3 _p2, ngl::Real _r1, ngl::Real _r2)
{
  // Calculate the direction of the collision
    ngl::Vec3 collisionNormal = _p1 - _p2;
    ngl::Real dist = collisionNormal.length();
    collisionNormal.normalize();

    // Calculate the overlap distance
    ngl::Real overlap = (_r1 + _r2) - dist;

    // Move the spheres away from each other along the collision normal
    ngl::Vec3 correction = collisionNormal * overlap * 0.5f;
    return correction;
}

ngl::Vec3 NGLScene::resolveBoxOverlap(ngl::Vec3 _planeNormal, ngl::Vec3 _pos, ngl::Real _rad)
{
  // Variable for overlap distance
  float d;

  // get overlap distance and offset the position of the sphere, to resolve the overlap with the box.
  // x axis
  if(_planeNormal.m_x != 0)
  {
    if(_planeNormal.m_x == 1)
    {
      d = (_pos.m_x + _rad) - m_bbox->maxX();
      _pos.m_x = _pos.m_x - d;
    }
    if(_planeNormal.m_x == -1)
    {
      d = m_bbox->minX() - (_pos.m_x - _rad);
      _pos.m_x = _pos.m_x + d;
    }
  }

  // y axis
  if(_planeNormal.m_y != 0)
  {
    if(_planeNormal.m_y == 1)
    {
      d = (_pos.m_y + _rad) - m_bbox->maxY();
      _pos.m_y = _pos.m_y - d;
    }
    if(_planeNormal.m_y == -1)
    {
      d =  m_bbox->minY() - (_pos.m_y - _rad);
      _pos.m_y = _pos.m_y + d;
    }
  }
   
  // z axis
  if(_planeNormal.m_z != 0)
  {
    if(_planeNormal.m_z == 1)
    {
      d = (_pos.m_z + _rad) - m_bbox->maxZ();
      _pos.m_z = _pos.m_z - d;
    }
    if(_planeNormal.m_z == -1)
    {
      d =  m_bbox->minZ() - (_pos.m_z - _rad);
      _pos.m_z = _pos.m_z + d;
    }
  }
  return _pos;
}


/* The box collision detection and ray tracing algorithm in this function is originally from 
   the NGL demo: BoundingBox written by Jon Macey. */
void NGLScene::bboxCollision()
{
 // create an array of the extents of the bounding box
  float ext[6];
  ext[0] = ext[1] = (m_bbox->height() / 2.0f);
  ext[2] = ext[3] = (m_bbox->width() / 2.0f);
  ext[4] = ext[5] = (m_bbox->depth() / 2.0f);
  
  // Vector to store sphere position
  ngl::Vec3 p;
  
  // distance between sphere and the extents of the box
  GLfloat D;

  // Loop for each sphere in the vector list
  for (Sphere &s : m_spheres)
  {
    bool collisionState = false;

    p = s.getPos();
    // Now we need to check the Sphere agains all 6 planes of the BBOx
    // If a collision is found we change the dir of the Sphere then Break
    for (int i = 0; i < 6; ++i)
    {
      // to calculate the distance we take the dotporduct of the Plane Normal
      // with the new point P
      D = m_bbox->getNormalArray()[i].dot(p);
      // Now Add the Radius of the sphere to the offsett
      D += s.getRadius();
      // If this is greater or equal to the BBox extent /2 then there is a collision
      // So we calculate the Spheres new direction
      if (D >= ext[i])
      {
        collisionState = true;
        // We use the same calculation as in raytracing to determine the
        //  the new direction
        GLfloat x = 2 * (s.getVel().dot((m_bbox->getNormalArray()[i])));
        ngl::Vec3 d = m_bbox->getNormalArray()[i] * x;
        
        s.setVel(s.getVel() - d);

        if(s.getVel().m_y != 0)
        {
          s.setVel(s.getVel() * (1 - m_energyLoss));
        }    
        
        ngl::Vec3 newPos = resolveBoxOverlap(m_bbox->getNormalArray()[i], s.getPos(), s.getRadius());
        s.setPos(newPos);

        //Make x and y velocities 0 if sphere reaches rest
        ngl::Vec3 vel = s.getVel();
        if(vel.length() < 0.002)
        {
            s.setVel(ngl::Vec3::zero());
        }
      }
    }  
    // Implement m_gravity when it's not colliding with the box 
    if(!collisionState)
    {
      s.gravity(m_gravity);
    } 

    // Check is sphere is coming to rest on the floor, and set the y to 0
    if(m_gravity.m_y <= 0)
    {
      s.checkForRest(m_bbox->minY(), m_gravity.m_y);
    }
    else 
    {
      s.checkForRest(m_bbox->maxY(), m_gravity.m_y);
    }

    // Implement friction when rolling in the ground
    if(s.getVel().m_y == 0)
    {
      s.friction(m_gravity, m_meu);
    }
  }    
}

void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_win.width,m_win.height);

  // Rotation based on the mouse position for our global transform
  auto rotX = ngl::Mat4::rotateX(m_win.spinXFace);
  auto rotY = ngl::Mat4::rotateY(m_win.spinYFace);

  // multiply the rotations
  m_mouseGlobalTX = rotX * rotY;
  // add the translations
  m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
  m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_y;
  m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;
  // get the VBO instance and draw the built in teapot
  // draw


  ngl::ShaderLib::use("nglColourShader");
  loadMatricesToColourShader();

  ngl::ShaderLib::use("nglDiffuseShader");
  
  for (Sphere &s : m_spheres)
  {
    s.draw("nglDiffuseShader", m_mouseGlobalTX, m_view, m_project); 
  } 
  
  // set the shaders for the bbox
  ngl::ShaderLib::setUniform("MVP",m_project * m_view);
  loadMatricesToShader();

  m_bbox->setDrawMode(GL_LINE);
  m_bbox->draw();
}

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
  case Qt::Key_Space :
      m_win.spinXFace=0;
      m_win.spinYFace=0;
      m_modelPos.set(ngl::Vec3::zero());

  break;
  default : break;
  }

  // finally update the GLWindow and re-draw
  update();
}

void NGLScene::updateSim()
{
  // sphere physics
  if(m_spheres.size() > 0)
  {
    for (int i = 0; i < m_spheres.size(); i++)
    {
      m_spheres[i].updatePosAndVel();
    }
  }
   
  bboxCollision();
  checkSphereCollisions();
}
void NGLScene::timerEvent(QTimerEvent *_event)
{
  if(m_runSim)
  {
    if (_event->timerId() == physics_timer)
    {
      updateSim();
    }
  }
  // re-draw GL
  update();
}
