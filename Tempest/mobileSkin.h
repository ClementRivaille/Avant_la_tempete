#ifndef MOBILE_SKIN_H
#define MOBILE_SKIN_H 1

#include "AReVi/Lib3D/object3D.h"
#include "weirdSkins.h"
#include "AReVi/Contrib/arMath.h"

namespace AReVi {
class Transform3D;


//---- Class Mobile ---------------------------------------------------------
class Mobile : public Object3D
{
public:

AR_CLASS(Mobile)
AR_CONSTRUCTOR(Mobile)

virtual
void action( double dt, Point3d target );

virtual
void action( double dt, Point3d target, ArConstRef<Transform3D> transform );

virtual
double
getMass() const;

public:
  Vector3d _speed;
  double _mass;
};

//---- Class MobilesSkin ----------------------------------------------------
class MobilesSkin : public Skin0
{
public:
  AR_CLASS(MobilesSkin)
  AR_CONSTRUCTOR(MobilesSkin)

  virtual
  void
  setParameters( unsigned int numberPerJoint = 500);

  virtual
  void
  setSpheresScale( double scaleFactor );

  virtual
  double 
  getSpheresScale() const;

  virtual
  bool
  createGeometries( ArRef<hLib::Skeleton> skeleton );

  virtual
  void
  update( double dt );

protected:
  struct Element {
    StlVector<ArRef<Mobile> > mobiles;
    StlVector<double> ratios;
    ArPtr<hLib::Joint> joint;
  };
  StlVector<Element> _elements;
  unsigned int _numberPerJoint;
  double _spheresScaleFactor;
  double _oldSpheresScaleFactor;
};

} //namespace AReVi

#endif //MOBILE_SKIN_H
