#ifndef WEIRD_SKINS_H
#define WEIRD_SKINS_H 1

#include "AReVi/arObject.h"

namespace hLib {
  class Skeleton;
  class Joint;
}

namespace AReVi {
class Scene3D;
class Object3D;
class Base3D;
class Cylinder3D;

class Skin0 : public ArObject {
public:
  AR_CLASS(Skin0)
  AR_CONSTRUCTOR(Skin0)

  virtual
  void
  setScene(ArRef<Scene3D> scene);

  virtual
  bool
  createGeometries( ArRef<hLib::Skeleton> skeleton );

  virtual
  void
  setVisible(bool visible);

  virtual
  bool
  getVisible() const;

  virtual
  void
  update( double dt );

//joint selection
  virtual
  bool
  selectJoint( const StlString & name);

//filtering the selected joints
  virtual
  void
  setFiltering(bool active);

protected:
  struct SelectedJoint
    {
    ArPtr<hLib::Joint> joint;
    StlVector<ArPtr<Base3D> > filterMemory;
    ArRef<Base3D> filterResult;
    };

  virtual
  void
  _computeFilter(SelectedJoint & selectedJoint);
  
protected:
  StlVector<ArRef<Object3D> > _objects; 
  bool _visible;
  ArPtr<Scene3D> _scene;
  StlVector<StlString> _selectedJointsNames;
  StlVector<SelectedJoint> _selectedJoints;
  bool _filter;
  unsigned int _nextFilter;
  unsigned int _filterSize;
  ArPtr<hLib::Skeleton> _skeleton; 
  
};

class TrihedronSkin : public Skin0 {
public:
  AR_CLASS(TrihedronSkin)
  AR_CONSTRUCTOR(TrihedronSkin)

  virtual
  bool
  createGeometries( ArRef<hLib::Skeleton> skeleton );
  
};

class SpheresSkin : public Skin0 {
public:
  AR_CLASS(SpheresSkin)
  AR_CONSTRUCTOR(SpheresSkin)

  virtual
  void
  setParameters(
    double radiusScale,
    double shapeScale = 1.0,
    unsigned int count = 50,
    double radiusMin = 1.0,
    double radiusMax = 2.0
  );

  virtual
  bool
  createGeometries( ArRef<hLib::Skeleton> skeleton );

  virtual
  void
  update( double dt );
protected:
  double _radiusScale;
  double _shapeScale;
  unsigned int _count;
  double _radiusMin;
  double _radiusMax;
};

class CylindersSkin : public Skin0 {
public:
  AR_CLASS(CylindersSkin)
  AR_CONSTRUCTOR(CylindersSkin)

  virtual
  void
  setRadius( double radius = 0.1);

  virtual
  double 
  getRadius() const;

  virtual
  bool
  createGeometries( ArRef<hLib::Skeleton> skeleton );

  virtual
  void
  update( double dt );

protected:
  struct Cylinder {
    ArRef<Cylinder3D> shape;
    ArRef<Object3D> object;
    ArPtr<hLib::Joint> joint;
  };

  virtual
  void
  _updateCylinder(Cylinder & cylinder);

  StlVector<Cylinder> _cylinders;

  double _radius;
};

} //namespace AReVi
#endif //WEIRD_SKINS 
