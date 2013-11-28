#include "tempestSkins.h"

#include "hLib/hLib.h"
#include "AReVi/Lib3D/object3D.h"
#include "AReVi/Lib3D/scene3D.h"
#include "AReVi/Shapes/shape3D.h"
#include "AReVi/Shapes/sphere3D.h"

using namespace AReVi;
using namespace hLib;
using std::cout;
using std::endl;

AR_CLASS_DEF(Skin0,ArObject)

Skin0::Skin0(ArCW & arCW)
  : ArObject(arCW),
  _visible(false)
{
}

Skin0::~Skin0()
{
}

void
Skin0::createGeometries(ArRef<Skeleton> skeleton)
{
cout<<"Nb joints "<<skeleton->getNbJoints()<<endl;
for (unsigned int i(0);i<skeleton->getNbJoints();++i)
  {
  ArRef<Joint> joint = skeleton->accessJoint(i);
  cout<<"joint "<<joint->getName()<<endl;
  ArConstRef<Joint> parent = joint->getParent();
  if (parent.valid())
    { cout<<" -> parent is "<<parent->getName()<<endl; }
  else
    { cout<<" -> no parent"<<endl; }

  ArRef<Object3D> object = Object3D::NEW();
  object->setLocation(joint);
  object->attachTo(joint);
  ArRef<Sphere3D> sphere = Sphere3D::NEW();
  sphere->setRadius(0.1);
  ArRef<Shape3D> shape = Shape3D::NEW();
  shape->addRootPart(sphere);
  object->setShape(shape); 
  _objects.push_back(object);
  cout<<"skin New object"<<endl;
  }
}

void
Skin0::setVisible( ArRef<Scene3D> scene, bool visible)
{
if (_visible && !visible)
  {
  for (unsigned int i(0);i<_objects.size();++i)
    { scene->removeObject(_objects[i]); }
  _visible = false;
  }
else if (!_visible && visible)
  {
  for (unsigned int i(0);i<_objects.size();++i)
    { scene->addObject(_objects[i]); }
  _visible = true;
  }
}

