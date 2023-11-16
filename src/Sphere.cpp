#include "Sphere.h"
#include "NGLScene.h"
#include <ngl/VAOPrimitives.h>
#include <ngl/NGLStream.h>
#include <ngl/ShaderLib.h>


Sphere::Sphere(ngl::Vec3 _pos, ngl::Vec3 _vel, float _radius, float _mass, ngl::Vec4 _colour)
{
    m_pos = _pos;
    m_vel = _vel;
    m_radius = _radius;
    m_mass = _mass;
    m_colour = _colour;
}

void Sphere::loadMatricesToShader( ngl::Transformation &_tx, const ngl::Mat4 &_globalMat,const  
                                    ngl::Mat4 &_view,const ngl::Mat4 &_project) 
{
    ngl::Mat4 MV;
    ngl::Mat4 MVP;
    ngl::Mat3 normalMatrix;
    MV=_view  *_globalMat* _tx.getMatrix();
    MVP=_project*MV;
    normalMatrix=MV;
    normalMatrix.inverse().transpose();
    ngl::ShaderLib::setUniform("MVP",MVP);
    ngl::ShaderLib::setUniform("normalMatrix",normalMatrix);
}

void Sphere::draw(const std::string &_shaderName, const ngl::Mat4 &_globalMat,  
                    const ngl::Mat4 &_view , const ngl::Mat4 &_project)
{
    m_tx.setScale(m_radius, m_radius, m_radius);

    ngl::ShaderLib::use(_shaderName);
    // grab an instance of the primitives for drawing

    ngl::ShaderLib::setUniform("Colour", m_colour);

    loadMatricesToShader(m_tx,_globalMat,_view,_project);
    ngl::VAOPrimitives::draw("sphere");
}

void Sphere::updatePos()
{
    m_tx.reset();
    m_tx.setPosition(m_pos);
}

void Sphere::updatePosAndVel()
{
    m_tx.reset();
    m_pos = m_pos + m_vel;
    m_tx.setPosition(m_pos);
}

void Sphere::checkForRest(ngl::Real _floorPos, float _gravity)
{
    double sphereTip;
    if(_gravity <= 0)
    {
        sphereTip = m_pos.m_y - m_radius;
    }
    else sphereTip = m_pos.m_y + m_radius;

    if((_floorPos-0.1f < sphereTip && sphereTip < _floorPos+0.1f))
    {
        if(-0.2f < m_vel.m_y && m_vel.m_y < 0.2f)
        {
            m_vel.m_y = 0;
        }
    }
}

void Sphere::friction(ngl::Vec3 _gravity, ngl::Real _meu)
{
    // Formula -> friction = -1 * meu * N * normVelVec

    // Calculate N
    ngl::Real normal = m_mass * _gravity.m_y;

    // Calculate normVelVec
    ngl::Vec3 normVelVec = m_vel;
    normVelVec.m_y = 0.0f;

    // Check if velocity isn't 0
    if(normVelVec != ngl::Vec3::zero())
    {
        normVelVec.normalize();
    }
    else
    {
        normVelVec = ngl::Vec3::zero();
    }
    
    // Calculate friction
    ngl::Vec3 friction = -1 * _meu * normal * normVelVec; 

    // Add friction offset to velocity
    m_vel = m_vel - friction;
}

void Sphere::gravity(ngl::Vec3 _gravity)
{
    m_vel = m_vel + _gravity;
}

void Sphere::setColour(ngl::Vec4 _colour)
{   
    m_colour = _colour;
}
