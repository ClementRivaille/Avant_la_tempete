#include "mobileSkin.h"

#include "AReVi/Shapes/shape3D.h"
#include "AReVi/Shapes/sphere3D.h"
#include "AReVi/Lib3D/transform3D.h"
#include "AReVi/arSystem.h"

#include "hLib/hLib.h"

//---- Class Mobile ---------------------------------------------------------
using namespace AReVi;
AR_CLASS_DEF(Mobile,Object3D)

Mobile::Mobile(ArCW & arCW)
  : Object3D(arCW),
  _speed(0.0,0.0,0.0),
  _mass(0.3+5.0*ArSystem::realRand())
{
setPosition(
  (-0.5 + ArSystem::realRand() ),
  (-0.5 + ArSystem::realRand() ),
  (-0.5 + ArSystem::realRand() ) );
_speed = 1 * (Vector3d(-0.5,-0.5,-0.5) + Vector3d(ArSystem::realRand(),ArSystem::realRand(),ArSystem::realRand()));
_speed = 0.1 * _speed;
}

Mobile::~Mobile()
{
}

void
Mobile::action(
  double dt, 
  Point3d target
)
{
//
Point3d p;
getPosition(p.x,p.y,p.z);

Vector3d delta(p,target);
const double distance = delta.length();

Vector3d speed = _speed;
if (distance > 0.01)
  {
  delta.normalize();
  Vector3d grav = 0.5 * _mass * delta / (distance*distance);

  const double dSeuil = 0.1;
  const double d2 = std::max(0.0,distance - dSeuil);
  _speed = _speed + grav*dt;
  double speedNorm = _speed.length();
  if( speedNorm > 0.2)
    { _speed = 0.2 * _speed / speedNorm; }

  Vector3d speed2 = delta * d2 * _mass;
  speed =  _speed + speed2;
  }
setPosition(p.x+speed.x()*dt,p.y+speed.y()*dt,p.z+speed.z()*dt);
}

void
Mobile::action(
  double dt, 
  Point3d target,
  ArConstRef<Transform3D> transform
)
{
action(dt,target);
accessShape()->accessRootPart(0)->writeTransformation(transform);
}

double
Mobile::getMass() const
{
return _mass;
}

//---- Class MobilesSkin ----------------------------------------------------

AR_CLASS_DEF(MobilesSkin,Skin0)

MobilesSkin::MobilesSkin(ArCW & arCW)
  : Skin0(arCW),
  _numberPerJoint(10),
  _spheresScaleFactor(0.4),
  _oldSpheresScaleFactor(0.4)
{
}

MobilesSkin::~MobilesSkin()
{
}

void
MobilesSkin::setParameters(unsigned int numberPerJoint)
{
_numberPerJoint = numberPerJoint;
}

void
MobilesSkin::setSpheresScale( double scaleFactor )
{
_spheresScaleFactor = scaleFactor;
}

double
MobilesSkin::getSpheresScale() const
{
return _spheresScaleFactor;
}

bool
MobilesSkin::createGeometries( ArRef<hLib::Skeleton> skeleton)
{
Skin0::createGeometries(skeleton);
ArRef<Transform3D> transform = Transform3D::NEW();
transform->preScale(_spheresScaleFactor, _spheresScaleFactor, _spheresScaleFactor);
for (unsigned int i(0);i<skeleton->getNbJoints();++i)
  {
  ArRef<Joint> joint = skeleton->accessJoint(i);
  Element element;
  _elements.push_back(element);
  _elements[i].joint = joint;
  unsigned int numberPerJoint = _numberPerJoint;
  double ratioStart = 0.0;
  const StlString name = joint->getName();
  if ((name == "l_shoulder") || (name == "r_shoulder"))
    { 
    numberPerJoint = 0.2 * _numberPerJoint; 
    ratioStart = 0.2;
    }
  for (unsigned int j(0);j<numberPerJoint;++j)
    {
    ArRef<Mobile> mobile = Mobile::NEW();
    _objects.push_back(mobile);

    ArRef<Sphere3D> sphere = Sphere3D::NEW();
    sphere->setRadius(mobile->getMass()/400.0);
    sphere->writeTransformation(transform);
    ArRef<Shape3D> shape = Shape3D::NEW();
    mobile->setShape(shape);
    shape->addRootPart(sphere);
    mobile->setMotionInteraction(false);  
    mobile->setMouseButtonInteraction(false);  
    mobile->setKeyboardInteraction(false);  
    mobile->setDetectMouse(false);  
    _elements[i].mobiles.push_back(mobile);
    double remains = 1.0 - ratioStart;
    _elements[i].ratios.push_back(ratioStart + remains* ArSystem::realRand());
    }
  }
return true;
}

void
MobilesSkin::update(double dt)
{
bool updateScale = _oldSpheresScaleFactor != _spheresScaleFactor;
ArRef<Transform3D> transform = Transform3D::NEW();
if (updateScale)
  { 
  _oldSpheresScaleFactor = _spheresScaleFactor;
  transform->preScale(_spheresScaleFactor, _spheresScaleFactor, _spheresScaleFactor);
  }
for (size_t i(0);i<_elements.size();++i)
  {
  Element & element = _elements[i];
  ArRef<Joint> parent = element.joint->accessParent();
  Point3d p;
  element.joint->getPosition(p.x,p.y,p.z);
  Point3d p2(p);
  if (parent.valid())
    { parent->getPosition(p2.x,p2.y,p2.z); }
  Vector3d v(p,p2);
  for (size_t j(0);j<element.mobiles.size();++j)
    { 
    Point3d p3(p);
    Vector3d v2 = v * element.ratios[j];
    p3.x = p3.x + v2.x();
    p3.y = p3.y + v2.y();
    p3.z = p3.z + v2.z();
    if (updateScale)
      { element.mobiles[j]->action(dt,p3,transform); }
    else
      { element.mobiles[j]->action(dt,p3); }
    }
  }
}

