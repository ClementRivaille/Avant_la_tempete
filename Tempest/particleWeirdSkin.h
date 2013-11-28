#ifndef PARTICLE_WEIRD_SKIN_H
#define PARTICLE_WEIRD_SKIN_H 1

#include "weirdSkins.h"

namespace hLib {
  class Joint;
}

namespace AReVi  {
class ParticleSystem;
class URLTexture;

class ParticleWeirdSkin : public Skin0
{
public:
  AR_CLASS(ParticleWeirdSkin)
  AR_CONSTRUCTOR(ParticleWeirdSkin)

public:
  virtual
  bool
  createGeometries( ArRef<hLib::Skeleton> skeleton );

  virtual
  void
  setVisible(bool visible);

  virtual
  void 
  update(double dt);

  virtual
  void
  setParticlesCount(unsigned int count);

  virtual
  void
  setParticlesScale(double scale);

  virtual
  double
  getParticlesScale() const;
protected:
  virtual
  void
  _setParticleLocation(unsigned int particle);

  virtual
  void
  _addParticle();

protected:
  struct Particle {
    ArRef<ParticleSystem> particleSystem;
    ArRef<hLib::Joint> joint;
  };
  StlVector<Particle> _particles;
  size_t _count;
  ArRef<URLTexture> _texture;
  unsigned int _parametersSet;
  double _scale;
  double _oldScale;
};

} //namespace AReVi
#endif //PARTICLE_WEIRD_SKIN_H
