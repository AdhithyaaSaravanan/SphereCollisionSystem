#ifndef SPHERE_H_
#define SPHERE_H_

#include <ngl/ShaderLib.h>
#include <ngl/Transformation.h>
#include <ngl/Vec3.h>



class Sphere
{
  public:

    //constructor
    Sphere() = default;
    Sphere(ngl::Vec3 _position, ngl::Vec3 _velocity, float _radius, float _mass, ngl::Vec4 _colour);

    //Methods
    // getters
    ngl::Vec3 getPos() const {return m_pos;}
    ngl::Vec3 getVel() const {return m_vel;}
    ngl::Real getRadius() const {return m_radius;}
    ngl::Real getMass() const {return m_mass;}
    ngl::Vec4 getColour() const {return m_colour;} 

    // setters
    void setPos(ngl::Vec3 const _pos) {m_pos = _pos;}
    void setVel(ngl::Vec3 const _vel) {m_vel = _vel;}
    void setMass(ngl::Real const _mass) {m_mass = _mass;}
    void setRadius(ngl::Real const _radius) {m_radius = _radius;}

    // functions
    void draw(const std::string &_shaderName, const ngl::Mat4 &_globalMat,  
              const ngl::Mat4 &_view , const ngl::Mat4 &_project);
    void updatePos();
    void updatePosAndVel();
    void gravity(ngl::Vec3 _gravity);
    void checkForRest(ngl::Real _floorPos, float _gravity);
    void friction(ngl::Vec3 _gravity, ngl::Real);
    void setColour(ngl::Vec4 _colour);
    void loadMatricesToShader(ngl::Transformation &_tx, const ngl::Mat4 &_globalMat,
                              const  ngl::Mat4 &_view,const ngl::Mat4 &_project);

  private:
    // Sphere attributes
    ngl::Vec3 m_pos;
    ngl::Vec3 m_vel;
    ngl::Real m_radius;
    ngl::Transformation m_tx;
    float m_mass;
    ngl::Vec4 m_colour;
};

#endif