#ifndef TEMPEST_SKINS_H
#define TEMPEST_SKINS_H 1

#include "AReVi/arObject.h"

namespace hLib {
class Skeleton;
}

namespace AReVi {
class Scene3D;
class Object3D;

class Skin0 : public ArObject {
public:
  AR_CLASS(Skin0)
  AR_CONSTRUCTOR(Skin0)

  virtual
  void
  createGeometries(ArRef<hLib::Skeleton> skeleton);

  virtual
  void
  setVisible(ArRef<Scene3D> scene, bool visible);

protected:
  StlVector<ArRef<Object3D> > _objects;
  bool _visible;
};

} // namespace AReVi
#endif //TEMPEST_SKINS_H
