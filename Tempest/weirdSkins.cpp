#include "weirdSkins.h"

#include "AReVi/Lib3D/scene3D.h"
#include "AReVi/Lib3D/base3D.h"
#include "AReVi/Lib3D/object3D.h"
#include "AReVi/Lib3D/transform3D.h"
#include "AReVi/Lib3D/material3D.h"
#include "AReVi/Shapes/shape3D.h"
#include "AReVi/Shapes/trihedron.h"
#include "AReVi/Shapes/sphere3D.h"
#include "AReVi/Shapes/cylinder3D.h"
#include "AReVi/arSystem.h"
#include "AReVi/Contrib/arMath.h"

#include "hLib/hLib.h"

using namespace AReVi;
using namespace hLib;
using std::cout;
using std::cerr;
using std::endl;

//---- Class Skin0 ----------------------------------------------------------
AR_CLASS_DEF(Skin0,ArObject)

Skin0::Skin0(ArCW & arCW)
  : ArObject(arCW),
  _visible(false),
  _filter(false),
  _nextFilter(0),
  _filterSize(5)
{
}

Skin0::~Skin0()
{
}

bool
Skin0::createGeometries(ArRef<Skeleton> skeleton)
{
_skeleton = skeleton;
return true;
}

void
Skin0::setScene(ArRef<Scene3D> scene)
{
_scene = scene;
}

void
Skin0::setVisible(bool visible)
{
if (!_scene.valid())
  { return; }

if (_visible && !visible)
  {
  for (unsigned int i(0);i<_objects.size();++i)
    { _scene->removeObject(_objects[i]); }
  _visible = false;
  }
else if (!_visible && visible)
  {
  for (unsigned int i(0);i<_objects.size();++i)
    { _scene->addObject(_objects[i]); }
  _visible = true;
  }
}

bool
Skin0::getVisible() const
{
return _visible;
}

void
Skin0::update(double) //dt
{
//filter evolution
if (_filter && !_selectedJoints.empty())
  {
  for (unsigned int i(0);i<_selectedJoints.size();++i)
    { _computeFilter(_selectedJoints[i]); }
  ++_nextFilter;
  if (_nextFilter>_filterSize)
    { _nextFilter = 0; }
  }
}

bool
Skin0::selectJoint( const StlString & name)
{
 _selectedJointsNames.push_back(name); 
return true; //TODO not boolean
}

void
Skin0::setFiltering( bool active)
{
_filter = active;
}

void
Skin0::_computeFilter(SelectedJoint & selectedJoint)
{
/* TODO or remove filter, on ne s'en sert pas en definitive
unsigned int f(_nextFilter);
for (i=0;i<_filterSize;++i)
  {
  if (f>_filterSize)
    { f = 0; }
  ++f;
  }
*/
}

//---- Class TrihedronSkin --------------------------------------------------
AR_CLASS_DEF(TrihedronSkin,Skin0)

TrihedronSkin::TrihedronSkin( ArCW & arCW )
  : Skin0(arCW)
{
}

TrihedronSkin::~TrihedronSkin()
{
}

bool
TrihedronSkin::createGeometries( ArRef<hLib::Skeleton> skeleton )
{
Skin0::createGeometries(skeleton);
if (!skeleton.valid())
  { 
  setErrorMessage("Init trihedron skin but invalid skeleton");
  return false;
  }
for (unsigned int i(0); i<skeleton->getNbJoints(); ++i)
  {
  ArRef<Object3D> object = Object3D::NEW();
  object->setShape(Trihedron::NEW(0.1));  
  _objects.push_back(object);
  ArRef<Joint> joint = skeleton->accessJoint(i);
  object->setLocation(joint);
  object->attachTo(joint);
  }
return true;
}

//---- Class SpheresSkin ----------------------------------------------------
AR_CLASS_DEF(SpheresSkin, Skin0)


SpheresSkin::SpheresSkin(ArCW & arCW)
  : Skin0(arCW),
  _radiusScale(1.0),
  _shapeScale(1.0),
  _count(50),
  _radiusMin(1.0),
  _radiusMax(1.0)
{
}

SpheresSkin::~SpheresSkin()
{
}

void
SpheresSkin::setParameters(
  double radiusScale,
  double shapeScale,
  unsigned int count,
  double radiusMin,
  double radiusMax
)
{
_radiusScale = radiusScale;
_shapeScale = shapeScale;
_count = count;
_radiusMin = radiusMin;
_radiusMax = radiusMax;
}

bool
SpheresSkin::createGeometries( ArRef<hLib::Skeleton> skeleton)
{
Skin0::createGeometries(skeleton);
ArRef<Transform3D> transform = Transform3D::NEW();

StlVector<ArRef<Joint> > joints;
if (_selectedJointsNames.empty())
  {
  for (unsigned int i(0);i<skeleton->getNbJoints();++i)
    { joints.push_back(_skeleton->accessJoint(i)); }
  }
else
  { 
  for (unsigned int i(0);i<_selectedJointsNames.size();++i)
    { 
    for (unsigned int j(0);j<skeleton->getNbJoints();++j)
      {
      ArRef<Joint> joint = skeleton->accessJoint(j);
      if (joint->getName() == _selectedJointsNames[i])
        { 
        SelectedJoint selectedJoint;
        selectedJoint.joint = joint;
        for (unsigned int k(0);k<_filterSize;++k)
          { selectedJoint.filterMemory.push_back(Base3D::NEW()); }
        selectedJoint.filterResult=Base3D::NEW();
         
        _selectedJoints.push_back(selectedJoint);
        joints.push_back(joint);
        cout<<"Create geometries sphere found selected joint : "
                 <<_selectedJointsNames[i]<<endl;
        }
      }
    }
  }

for (unsigned int i(0);i<joints.size();++i)
  {
  ArRef<Joint> joint = joints[i];
  ArRef<Joint> parent = joint->accessParent();
  if (parent.valid())
    {
    cout<<"Spheres skin add segment"<<endl;
    ArRef<Object3D> object = Object3D::NEW();
    ArRef<Shape3D> shape = Shape3D::NEW();
    object->setShape(shape);
    _objects.push_back(object);
    Point3d j;
    joint->getPosition(j.x,j.y,j.z);
    parent->globalToLocalPosition(j.x,j.y,j.z);
    Point3d p;
    parent->getPosition(p.x,p.y,p.z);
    parent->globalToLocalPosition(p.x,p.y,p.z);
    Vector3d v(p,j);
    if (v.length()>0.0)
      {
      for (unsigned int j(0);j<_count;++j)
        {
        ArRef<Material3D> material = Material3D::NEW();
        double c = ArSystem::realRand();
        double e = ArSystem::realRand();
        material->setDiffuseColor(c,1.0,1.0);
        material->setEmissiveColor(1.0,e,e);
        ArRef<Sphere3D> sphere = Sphere3D::NEW();
        sphere->writeMaterial(material);
        sphere->setRadius(
          _radiusScale*(_radiusMin + ArSystem::realRand()*(_radiusMax - _radiusMin))
        );
        transform->identity();
        Vector3d t = v * ArSystem::realRand();
        Vector3d offset(
          -0.5 + ArSystem::realRand(),
          -0.5 + ArSystem::realRand(),
          -0.5 + ArSystem::realRand());
        t = t + offset * _shapeScale;
        transform->preTranslate(t.x(),t.y(),t.z()); 
        
        sphere->writeTransformation(transform);
        shape->addRootPart(sphere);
        }
      }
    object->setLocation(parent);
    object->attachTo(parent);
    }
  else 
    { cout<<"Joint "<<joint->getName()<<" without parent"<<endl; }
  }
return true;
}

void
SpheresSkin::update( double dt)
{
Skin0::update(dt);
}

//---- Class CylindersSkin ----------------------------------------------------

AR_CLASS_DEF(CylindersSkin,Skin0)

CylindersSkin::CylindersSkin( ArCW & arCW )
  : Skin0(arCW),
  _radius(0.004)
{
}

CylindersSkin::~CylindersSkin()
{
}

void
CylindersSkin::setRadius( double radius )
{
_radius = radius;
}


double
CylindersSkin::getRadius() const
{
return _radius;
}


bool
CylindersSkin::createGeometries( ArRef<hLib::Skeleton> skeleton )
{
Skin0::createGeometries(skeleton);
for (unsigned int i(0);i<skeleton->getNbJoints();++i)
  {
  ArRef<Joint> joint = skeleton->accessJoint(i);
  ArRef<Joint> parent = joint->accessParent();
  if (parent.valid())
    {
    Cylinder cylinder;
    cylinder.shape = Cylinder3D::NEW();
    cylinder.object = Object3D::NEW();
    cylinder.joint = joint;
    _cylinders.push_back(cylinder);
    ArRef<Shape3D> shape = Shape3D::NEW();
    shape->addRootPart(cylinder.shape);
    cylinder.object->setShape(shape);
    _objects.push_back(cylinder.object);
    _updateCylinder(cylinder);
    }
  }
 return true;
}

void
CylindersSkin::update( double ) //dt
{
for (size_t i(0);i<_cylinders.size();++i)
  { _updateCylinder(_cylinders[i]); }
}
  
void
CylindersSkin::_updateCylinder(Cylinder & cylinder)
{
cylinder.object->setLocation(cylinder.joint);
ArRef<Joint> parent = cylinder.joint->accessParent();
if (parent.valid())
  {
  double x,y,z;
  parent->getPosition(x,y,z);
  cylinder.object->globalToLocalPosition(x,y,z);
  double distance = sqrt(x*x + y*y + z*z);
  ArRef<Transform3D> transform = Transform3D::NEW();
  transform->preTranslate(0.0, 0.0, distance*0.5);
  cylinder.shape->writeTransformation(transform);
  cylinder.shape->setRadius(_radius);
  cylinder.shape->setHeight(distance);
  Vector3d start(0.0,0.0,1.0);
  Vector3d goal(x,y,z);
  goal.normalize();
  Vector3d cross = start ^ goal; 
  const double angle = start.angle(goal); 
  cross.normalize();
  double vx = cross.x();
  double vy = cross.y();
  double vz = cross.z();
  if (cross.length() != 0.0)
    { cylinder.object->rotate(vx,vy,vz,angle); }
  }
}


