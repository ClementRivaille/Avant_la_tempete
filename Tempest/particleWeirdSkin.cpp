//TODO set doivent etres independants de l'ordre d'appel setScene, Count etc
#include "particleWeirdSkin.h"

#include "AReVi/Lib3D/particleSystem.h"
#include "AReVi/Lib3D/urlTexture.h"
#include "AReVi/Lib3D/scene3D.h"
#include "AReVi/arSystem.h"

#include "hLib/hLib.h"

using namespace hLib;
using namespace AReVi;
using std::cout;
using std::endl;
using std::cerr;

AR_CLASS_DEF(ParticleWeirdSkin,Skin0)

ParticleWeirdSkin::ParticleWeirdSkin(
  ArCW & arCW
)
  :Skin0(arCW),
  _count(0),
  _texture(URLTexture::NEW("data/lightSmoke.png",false,false)),
  _parametersSet(2),
  _scale(0.02),
  _oldScale(0.02)
{
if (_texture->fail())
  { cerr<<"Texture load fail :"<<_texture->getErrorMessage()<<endl; }
}

ParticleWeirdSkin::~ParticleWeirdSkin()
{
}

bool
ParticleWeirdSkin::createGeometries( ArRef<Skeleton> skeleton)
{
Skin0::createGeometries(skeleton);
_skeleton = skeleton;
if (_skeleton.valid())
  {
  unsigned int count = std::min(_count,_particles.size());
  for (unsigned int i(0);i<count;++i)
    { _setParticleLocation(i); }
  return true;
  }
return false;
}

void
ParticleWeirdSkin::setVisible(bool visible)
{
cout<<"Set visible"<<endl;
if (!_scene.valid())
  { return; }

unsigned int count = std::min(_particles.size(),_count);
cout<<"-> count "<<count<<endl;
if (_visible && !visible)
  {
  for (unsigned int i(0);i<count;++i)
    { _scene->removeParticleSystem(_particles[i].particleSystem); }
  }
else if (!_visible && visible)
  {
  for (unsigned int i(0);i<count;++i)
    { _scene->addParticleSystem(_particles[i].particleSystem); }
  }
_visible = visible;
}

void
ParticleWeirdSkin::update( double dt)
{
unsigned int count = std::min(_count,_particles.size());
//cout<<"Particle skin update dt:"<<dt<<" particles "<<_particles.size()<<endl;
if (_oldScale != _scale)
  {
  _oldScale = _scale;
  for (size_t i(0);i<_particles.size();++i)
    {
    double sizeRand = ArSystem::realRand()*_scale;
    _particles[i].particleSystem->setSize(0.03+sizeRand,0.03+sizeRand);
    }
  }
for (unsigned int i(0);i<count;++i)
  { 
  _particles[i].particleSystem->update(dt); 
  _setParticleLocation(i);
  }
}

void
ParticleWeirdSkin::setParticlesCount(unsigned int count)
{
if (!_scene.valid()) 
  { return; }//TODO check add without scene
if (count<_count)
  {
  if (_visible)
    {
    for (unsigned int i=count;i<_count;++i)
      { _scene->removeParticleSystem(_particles[i].particleSystem); }  
    }
  }
else if (count>_count)
  {
  if (count>_particles.size())
    { 
    for (unsigned int i=_particles.size();i<count;++i)
      { _addParticle(); }
    }

  if (_skeleton.valid())
    {
    for (unsigned int i=_count;i<count;++i)
      {
      _setParticleLocation(i);
      if (_visible)
        { _scene->addParticleSystem(_particles[i].particleSystem); }
      }
    }
  }
_count = count;
}

void
ParticleWeirdSkin::setParticlesScale( double scale )
{
_scale = scale;
}

double
ParticleWeirdSkin::getParticlesScale() const
{
return _scale;
}

//forbidden methods
void
ParticleWeirdSkin::_setParticleLocation(unsigned int particle)
{
if (!_skeleton.valid())
  { return; }

//cout<<"Set particle location "<<particle<<endl;
unsigned int iJoint = ArSystem::integerRand(_skeleton->getNbJoints());


ArRef<Joint> target;
ArRef<Joint> joint = _skeleton->accessJoint(iJoint);
ArRef<Joint> parent = joint->accessParent();
if (parent.valid())
  {
  _particles[particle].particleSystem->setLocation(parent);//for the orientation
  double x1,y1,z1; 
  parent->getPosition(x1,y1,z1);
  double x2,y2,z2;
  joint->getPosition(x2,y2,z2);
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double dz = z2 - z1;
  const double f = ArSystem::realRand();
  _particles[particle].particleSystem->setPosition(x1+dx*f, y1+dy*f, z1+dz*f);
  target = parent;
  }
else 
  {
  _particles[particle].particleSystem->setLocation(joint);
  target = joint;

  }
  
//then translate "around"
const double l = 0.0;
const double a1 = ArSystem::realRand() * 2 * M_PI;
double x0 = cos(a1)*l;
double z = sin(a1)*l;
const double a2 = ArSystem::realRand() * 2 * M_PI;
double x = x0 * cos(a2);
double y = x0 * sin(a2);
_particles[particle].particleSystem->translate(x,y,z);

//check and change attach if needed
if (_particles[particle].joint != target)
  {
  if (_particles[particle].joint.valid())
    {
    _particles[particle].particleSystem->attachTo(
                          _particles[particle].joint,Base3D::ATTACH_NONE);
    }
  _particles[particle].particleSystem->attachTo(target);
  _particles[particle].joint = target;
  }
}

void
ParticleWeirdSkin::_addParticle()
{
//cout<<"Add particle"<<endl;
if (_parametersSet == 1)
  {
ArRef<ParticleSystem> ps = ParticleSystem::NEW();
ps->setSection(0.1,0.1,0.0,0.0,true);
ps->setEmissionSpeed(0.01);
ps->setEmissionSpeedDispersion(0.001);
ps->setInitialAngleDispersion(M_PI);
ps->setParticleDuration(0.1);
ps->setParticleRate(0.1);
ps->setLighting(false);
double sizeRand = ArSystem::realRand()*0.02;
ps->setSize(0.03+sizeRand,0.03+sizeRand);
ps->setSizeDispersion(0.0,0.0);
ps->setColor(1.0,1.0,1.0,1.0);
ps->setGravity(-0.001+ArSystem::realRand()*0.02,
               -0.001+ArSystem::realRand()*0.02,
               -0.001+ArSystem::realRand()*0.02);
ps->setTexture(_texture); 
Particle particle;
particle.particleSystem = ps;
particle.joint = Joint::nullRef();
_particles.push_back(particle);
  }
else if (_parametersSet == 2)
  {
ArRef<ParticleSystem> ps = ParticleSystem::NEW();
ps->setSection(0.1,0.1,0.0,0.0,true);
//ps->setEmissionSpeed(0.01);
ps->setEmissionSpeed(0.001);
ps->setEmissionSpeedDispersion(0.001);
ps->setInitialAngleDispersion(M_PI);
//ps->setParticleDuration(0.1+ArSystem::realRand()*0.3);
ps->setParticleDuration(0.1+ArSystem::realRand()*0.1);
//ps->setParticleRate(0.1);
ps->setParticleRate(0.01);
ps->setLighting(false);
double sizeRand = ArSystem::realRand()*_scale;
ps->setSize(0.03+sizeRand,0.03+sizeRand);
ps->setSizeDispersion(0.0,0.0);
ps->setColor(1.0,1.0,1.0,1.0);
ps->setGravity(-0.001+ArSystem::realRand()*0.02,
               -0.001+ArSystem::realRand()*0.02,
               -0.001+ArSystem::realRand()*0.02);
ps->setTexture(_texture); 
Particle particle;
particle.particleSystem = ps;
particle.joint = Joint::nullRef();
_particles.push_back(particle);
  }
}

