#ifndef TESTLOAD
#define TESTLOAD

#include <fstream>
#include <sstream>

#include <AReVi/Lib3D/viewer3D.h>
#include <AReVi/Lib3D/scene3D.h>
#include <AReVi/Lib3D/simpleInteractor.h>
#include <AReVi/Lib3D/boundingBox3D.h>
#include <AReVi/activity.h>
#include <AReVi/arSystem.h>
#include "hLib/hLib.h"
#include "weirdSkins.h"

using namespace AReVi;
using namespace hLib;

class App : public Viewer3D {
public:
  AR_CLASS(App)
  AR_CONSTRUCTOR(App)

protected:
  virtual void _initSkins();
  virtual bool _action(ArRef<Activity> act, double dt);


  ArRef<Scene3D> _scene;
  ArRef<SimpleInteractor> _interactor;
  ArRef<Body> _body;
  ArRef<KeyframeAnimation> _animation;
  ArRef<Skeleton> _skeleton;
  StlVector<ArRef<Skin0> > _skins;
};

#endif // TESTLOAD
