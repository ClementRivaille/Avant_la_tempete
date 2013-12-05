#include <pthread.h>>
#include <sys/socket.h>
#include <netdb.h>
#include <assert.h>
#include <string.h>

#include "AReVi/arSystem.h"
#include "AReVi/Lib3D/viewer3D.h"
#include "AReVi/Lib3D/simpleInteractor.h"
#include "AReVi/Lib3D/scene3D.h"
#include "AReVi/Lib3D/osdText.h"
#include "AReVi/Lib3D/object3D.h"
#include "AReVi/Lib3D/light3D.h"
#include "AReVi/Lib3D/particleSystem.h"
#include "AReVi/Lib3D/urlTexture.h"
#include "AReVi/Shapes/lineSet3D.h"
#include "AReVi/Shapes/trihedron.h"
#include "AReVi/Shapes/solidOfRevolution.h"
#include "AReVi/Utils/tcpUdp.h"
#include "AReVi/Utils/memoryBlock.h"
#include "AReVi/Utils/memoryStream.h"
#include "AReVi/Contrib/xmlParser.h"
#include "AReVi/Contrib/arMath.h"
#include "hLib/hLib.h"
#include "ArWiimote/ArWiiPassive.h"

#include "ArKinect2/kinectOpenNI2.h"

#include "udpKinectSkeleton.h"
#include "weirdSkins.h"
#include "particleWeirdSkin.h"
#include "mobileSkin.h"


using namespace std;
using namespace AReVi;
using namespace hLib;
// Utilities ------------------------------------------------------------------

void
kinectVolumePoint(double l, double yAngle, double zAngle, 
                  double & x, double & y, double & z)
{
Vector3d v(1.0,tan(zAngle),tan(yAngle));
v.normalize();
v =  v*l;
x = v.x();
y = v.y();
z = v.z();
}

ArRef<LineSet3D>
buildKinectVolume(unsigned int precision)
{
if (precision<1)
  { return LineSet3D::nullRef(); }
ArRef<LineSet3D> lineSet = LineSet3D::NEW();
StlVector<StlVector<Util3D::Dbl3> > & vertices = lineSet->accessVertices();
StlVector<Util3D::Dbl3> line;
StlVector<StlVector<Util3D::Dbl3> > & colors = lineSet->accessColors();
StlVector<Util3D::Dbl3> lineColors;
const double near = 0.8;
const double far = 3.5;
const double rightToLeft = 57.0/180.0*M_PI;
const double left = rightToLeft/2.0;
const double right = -rightToLeft/2.0;
const double bottomToTop = 45.0/180.0*M_PI;
const double bottom = -bottomToTop/2.0;
const double top = bottomToTop/2.0;

const Util3D::Dbl3 color(1.0,1.0,1.0);
//lines

//top right
double x,y,z; 
kinectVolumePoint(near,top,right,x,y,z);
line.push_back(Util3D::Dbl3(x,y,z));    lineColors.push_back(color);
kinectVolumePoint(far,top,right,x,y,z);
line.push_back(Util3D::Dbl3(x,y,z));    lineColors.push_back(color);
vertices.push_back(line);               colors.push_back(lineColors);

//top left
line.clear();
kinectVolumePoint(near,top,left,x,y,z);
line.push_back(Util3D::Dbl3(x,y,z));    lineColors.push_back(color);
kinectVolumePoint(far,top,left,x,y,z);
line.push_back(Util3D::Dbl3(x,y,z));    lineColors.push_back(color);
vertices.push_back(line);               colors.push_back(lineColors);

//bottom left
line.clear();
kinectVolumePoint(near,bottom,left,x,y,z);
line.push_back(Util3D::Dbl3(x,y,z));    lineColors.push_back(color);
kinectVolumePoint(far,bottom,left,x,y,z);
line.push_back(Util3D::Dbl3(x,y,z));    lineColors.push_back(color);
vertices.push_back(line);               colors.push_back(lineColors);

//bottom right 
line.clear();
kinectVolumePoint(near,bottom,right,x,y,z);
line.push_back(Util3D::Dbl3(x,y,z));    lineColors.push_back(color);
kinectVolumePoint(far,bottom,right,x,y,z);
line.push_back(Util3D::Dbl3(x,y,z));    lineColors.push_back(color);
vertices.push_back(line);               colors.push_back(lineColors);

//'horizontal' arcs
StlVector<Util3D::Dbl3> nearBottom;
StlVector<Util3D::Dbl3> farBottom;
StlVector<Util3D::Dbl3> farTop;
StlVector<Util3D::Dbl3> nearTop;
lineColors.clear();
for (unsigned int i(0);i<=precision;++i)
  {
  const double zAngle = right + rightToLeft * double(i)/double(precision);
  kinectVolumePoint(near,bottom,zAngle,x,y,z);
  nearBottom.push_back(Util3D::Dbl3(x,y,z)); 
  kinectVolumePoint(far,bottom,zAngle,x,y,z);
  farBottom.push_back(Util3D::Dbl3(x,y,z)); 
  kinectVolumePoint(far,top,zAngle,x,y,z);
  farTop.push_back(Util3D::Dbl3(x,y,z)); 
  kinectVolumePoint(near,top,zAngle,x,y,z);
  nearTop.push_back(Util3D::Dbl3(x,y,z)); 
  lineColors.push_back(color);
  }
vertices.push_back(nearBottom);
colors.push_back(lineColors);
vertices.push_back(farBottom);
colors.push_back(lineColors);
vertices.push_back(farTop);
colors.push_back(lineColors);
vertices.push_back(nearTop);
colors.push_back(lineColors);

//vertical arcs
StlVector<Util3D::Dbl3> nearLeft;
StlVector<Util3D::Dbl3> farLeft;
StlVector<Util3D::Dbl3> farRight;
StlVector<Util3D::Dbl3> nearRight;
for (unsigned int i(0);i<=precision;++i)
  {
  const double yAngle = bottom + bottomToTop * double(i)/double(precision);
  kinectVolumePoint(near,yAngle,left,x,y,z);
  nearLeft.push_back(Util3D::Dbl3(x,y,z)); 
  kinectVolumePoint(far,yAngle,left,x,y,z);
  farLeft.push_back(Util3D::Dbl3(x,y,z)); 
  kinectVolumePoint(far,yAngle,right,x,y,z);
  farRight.push_back(Util3D::Dbl3(x,y,z)); 
  kinectVolumePoint(near,yAngle,right,x,y,z);
  nearRight.push_back(Util3D::Dbl3(x,y,z)); 
  }
vertices.push_back(nearLeft);
colors.push_back(lineColors);
vertices.push_back(farLeft);
colors.push_back(lineColors);
vertices.push_back(farRight);
colors.push_back(lineColors);
vertices.push_back(nearRight);
colors.push_back(lineColors);

//
if (!lineSet->applyChanges(true))
  { cerr<<"Line set build error"<<endl; }
return lineSet;
}

//MultiReceiver ---------------------------------------------------------------

class MultiReceiver : public ArObject {
public:
  AR_CLASS(MultiReceiver)
  AR_CONSTRUCTOR(MultiReceiver)

public:
  virtual
  void
  setRenderer(ArRef<Renderer3D> renderer);

  virtual
  void
  addSource(ArRef<UDPKinectSkeleton> receiver, ArConstRef<Base3D> offset);

  virtual
  void
  addOSCTarget(const StlString & ip, unsigned int port, unsigned int port2);

  virtual 
  void
  createOSCTransmitter(unsigned int port);

protected:
  virtual
  void
  _initSkinsIfNeeded();

  virtual
  bool
  _action(ArRef<Activity> activity, double dt);

//feedback
  virtual 
  void 
  _printMessage(const StlString & text, double time);

  virtual
  void
  _setShowSourceDebug(unsigned int number, bool show);   

//control
  virtual
  void
  _keyboardCB(const Renderer3D::KeyboardEvent & evt);

  virtual
  void
  _switchSkinVisible(unsigned int number);

  virtual
  void
  _switchObjectSkin();

  virtual
  void
  _setSourceVolumeVisible(unsigned int number,bool visible);

  virtual
  void
  _computeSpeed();

  virtual
  void
  _catchObject();

  virtual
  void
  _releaseObject();

  virtual
  void
  _centerObject();

  virtual
  void
  _OSCReEmission();

  //TODO this is a bad copy paste from UDPKinectSkeleton
  virtual
  void
  _writeStringOSC(const StlString & value,
                                   ArRef<AbstractOStream> stream);

  virtual
  void
  _recordAnimationIfNeeded(double dt);
 
  virtual
  void
  _animationControlEvent(unsigned int slot, bool recordEvent, bool eraseEvent);

  virtual
  void
  _saveAnimations();

  virtual
  void
  _loadAnimations();

  virtual
  void
  _saveAnimation(
    ArRef<KeyframeAnimation> animation,
    const StlString & filename
  );

  virtual
  void
  _loadAnimation(size_t slot, const StlString & filename);

  //
  virtual
  void
  _initTempest();

  virtual
  void
  _switchTempestVisible();

  virtual
  void
  _tempestFollow(ArConstRef<Base3D> target, double dt);

protected:
  ArPtr<Renderer3D> _renderer;
  
  ArRef<OSDText> _osdText;
  double _osdTimer;
  
  struct Source {
    ArRef<UDPKinectSkeleton> receiver;
    StlVector<ArRef<Object3D> > trihedrons;
    ArRef<Base3D> offset;
    ArRef<Object3D> kinectVolume;
    bool showVolume;
    bool visible;
    bool showDebug;
    unsigned long int lastApparition;
  };
  StlVector<Source> _sources;
  int _lastSource;

  ArRef<Skeleton> _skeleton;
  ArRef<Skeleton> _skeletonForHandle;

  unsigned long int _frame;

  StlVector< ArRef<Skin0> > _skins;
  bool _initSkinsDone;
  bool _showSources;

  ArRef<ParticleWeirdSkin> _particlesSkin;
  ArRef<MobilesSkin> _mobilesSkin;
  ArRef<CylindersSkin> _cylindersSkin;

  ArRef<SpheresSkin> _galaxySkin;

  //quick and dirty : uses a skeleton as an object
  ArRef<Skeleton> _skeletonObject;
  ArRef<SpheresSkin> _objectSkin;
  ArRef<Base3D> _objectHandle;
  bool _objectHold;

  struct Delta {
    double x;
    double y;
    double z; 
    double roll;
    double pitch;
    double yaw;
    double t;
  };
  StlVector<Delta> _deltas; 
  unsigned int _nbDeltas;
  unsigned int _nextDelta;
  double _lastX;
  double _lastY;
  double _lastZ;
  double _lastRoll;
  double _lastPitch;
  double _lastYaw;
  Vector3d _objectSpeed;
  Vector3d _objectRotationSpeed;

  ArRef<ArWiiPassive> _wiimote;
  int _wantWiimote;

  bool _freeze;

  //recordings of skeleton
  StlVector<ArRef<KeyframeAnimation> > _animations;
  enum AnimationState { PLAY, RECORD, NONE };
  AnimationState _animationState;
  unsigned int _animationSlot;
  double _animationTimePosition;

  //re emission of an OSC message
  ArRef<UDPTransmitter> _udp;
  struct Target {
    unsigned int ip;
    unsigned int port; //relative to zero, aka camera position
    unsigned int port2; //relative to pelvis
  };
  StlVector<Target> _targetsOSC;

  //tempest : particles relative to the renderer
  ArRef<ParticleSystem> _tempest;
  ArRef<URLTexture> _tempestTexture;
  double _tempestEmissionSpeed;
  bool _tempestVisible;
  double _tempestX;
  bool _tempestFollowing;
  double _tempestYaw;
  double _tempestPitch;
  double _tempestChangeDelay;
};


AR_CLASS_DEF(MultiReceiver,ArObject)

MultiReceiver::MultiReceiver(ArCW & arCW)
: ArObject(arCW),
  _osdText(OSDText::NEW()),
  _osdTimer(0.0), 
  _lastSource(-1),
  _skeleton(Skeleton::nullRef()),
  _skeletonForHandle(Skeleton::nullRef()),
  _frame(0),
  _initSkinsDone(false),
  _showSources(true),
  _skeletonObject(Skeleton::NEW()),
  _objectHandle(Base3D::nullRef()),
  _objectHold(false),
  _nbDeltas(5),
  _nextDelta(0),
  _lastX(0.0),
  _lastY(0.0),
  _lastZ(0.0),
  _lastRoll(0.0),
  _lastPitch(0.0),
  _lastYaw(0.0),
  _wantWiimote(0),
  _freeze(false),
  _animations(12,KeyframeAnimation::nullRef()), //12 fn keys, empty slots
  _animationState(NONE),
  _animationSlot(0),
  _animationTimePosition(0.0),
  _tempest(ParticleSystem::NEW()),
  _tempestVisible(true),
  _tempestFollowing(false),
  _tempestYaw(0.0),
  _tempestPitch(0.0),
  _tempestChangeDelay(0.01)
{
ArRef<Activity> act = Activity::NEW(0.01);
act->setBehavior(thisRef(), &MultiReceiver::_action);
_osdText->setLocation(OSD::LOC_TOP_CENTER);

//create skeleton via kinect to have the same joints and parents
//ArRef<KinectOpenNI2> kinect = KinectOpenNI2::NEW();
//_skeleton = kinect->createPlayerSkeleton();
_skeleton = KinectOpenNI2::createASkeleton();
_skeletonForHandle = Skeleton::NEW(_skeleton);

ArRef<Joint> joint0 = Joint::NEW();
joint0->setName("zero");
joint0->setPosition(0.0,0.0,-0.25);
_skeletonObject->addJoint(joint0);

ArRef<Joint> joint1 = Joint::NEW();
joint1->setName("un");
joint1->setPosition(0.0,0.0,0.25);
joint1->setParent(joint0);
_skeletonObject->addJoint(joint1);
}

MultiReceiver::~MultiReceiver()
{
}

void
MultiReceiver::setRenderer( ArRef<Renderer3D> renderer )
{
_renderer = renderer;
if (_renderer.valid())
  { 
  _renderer->addKeyboardCB(thisRef(), &MultiReceiver::_keyboardCB);

  ArRef<Window3D> w=ar_down_cast<Window3D>(_renderer);
  if (w.valid())
    { w->setKeyRepeat(true); }
  //skins
  ArRef<Skin0> skin = TrihedronSkin::NEW(); 
  skin->setScene(_renderer->accessScene());
  _skins.push_back(skin);

  _galaxySkin = SpheresSkin::NEW();
  _galaxySkin->setParameters(0.5,3.0,200,0.001,0.025);
  _galaxySkin->setScene(_renderer->accessScene());
  _galaxySkin->selectJoint("neck");
  _galaxySkin->selectJoint("l_elbow");
  _skins.push_back(_galaxySkin);

  ArRef<SpheresSkin> spheresSkin = SpheresSkin::NEW();
  //spheresSkin->setParameters(0.5,0.1,5,0.05,0.15);

  spheresSkin->setParameters(1.0,0.01,50,0.005,0.005);

  spheresSkin->setScene(_renderer->accessScene());
  _skins.push_back(spheresSkin);

  _particlesSkin = ParticleWeirdSkin::NEW();
  _particlesSkin->setScene(_renderer->accessScene());
  _particlesSkin->setParticlesCount(100);
  _skins.push_back(_particlesSkin);

  _cylindersSkin = CylindersSkin::NEW();
  _cylindersSkin->setRadius(0.004); //radius  
  _cylindersSkin->setScene(_renderer->accessScene());
  _skins.push_back(_cylindersSkin);
  cout<<"Ajout de cylinders skins"<<endl;

  _mobilesSkin = MobilesSkin::NEW();
  _mobilesSkin->setParameters(100); //number per joint
  _mobilesSkin->setScene(_renderer->accessScene());
  _skins.push_back(_mobilesSkin);
  cout<<"Ajout de mobiles skin"<<endl;

  //
  _initTempest();

  //skeleton object
  _objectSkin = SpheresSkin::NEW();
  _objectSkin->setParameters(0.5,20.0,2000,0.002,0.1);
  _objectSkin->setScene(_renderer->accessScene());

  for (unsigned int i(0);i<_nbDeltas;++i)  
    {
    Delta delta;
    delta.x=0.0;
    delta.y=0.0;
    delta.z=0.0;
    delta.t=0.0;
    delta.roll=0.0;
    delta.pitch=0.0;
    delta.yaw=0.0;
    _deltas.push_back(delta);
    }
  }
}

void
MultiReceiver::_initSkinsIfNeeded()
{
if (_initSkinsDone)
  { return; }
if (_renderer.valid())
  {
  ArRef<Scene3D> scene = _renderer->accessScene();
  cout<<"Skin create geometries"<<endl;
  for (unsigned int i(0);i<_skins.size();++i)
    {
    ArRef<Skin0> skin0 = _skins[i];
    if (skin0->createGeometries(_skeleton))
      {
      cout<<"Set visible skin "<<i<<endl;
  
      skin0->setVisible(true);
      }
    else
      { cerr<<skin0->getErrorMessage()<<endl; }
    }

  _objectSkin->createGeometries(_skeletonObject);
  _objectSkin->setVisible(true);

  _objectHandle = _skeletonForHandle->findJoint("torso");
  _skeletonObject->findJoint("zero")->setLocation(_objectHandle);
  if (_objectHandle.valid())
    { _objectHandle->getPosition(_lastX,_lastY,_lastZ); }
  _initSkinsDone = true;
  }
}

void
MultiReceiver::addSource(
  ArRef<UDPKinectSkeleton> receiver,
  ArConstRef<Base3D> offset
)
{
Source source;
source.receiver = receiver;
source.offset = Base3D::NEW();
source.visible = false;
source.showDebug = false;
source.lastApparition= 0;
if (offset.valid())
  { source.offset->setLocation(offset); }
for (unsigned int i(0);i<receiver->getSkeleton()->getNbJoints();++i)
  {
  ArRef<Object3D> object = Object3D::NEW();
  ArRef<Shape3D> shape = Trihedron::NEW(0.1);
  object->setShape(shape);
  source.trihedrons.push_back(object);
  object->setLocation(receiver->getSkeleton()->getJoint(i));
  }

source.kinectVolume = Object3D::NEW();
ArRef<Shape3D> shape = Shape3D::NEW();
source.kinectVolume->setShape(shape);
ArRef<LineSet3D> lineSet = buildKinectVolume(24);
shape->addRootPart(lineSet);
source.kinectVolume->setLocation(source.offset);
source.kinectVolume->attachTo(source.offset);
if (_renderer.valid())
  { _renderer->accessScene()->addObject(source.kinectVolume); }

_sources.push_back(source);
}

void
MultiReceiver::addOSCTarget(
  const StlString & ipString,
  unsigned int port,
  unsigned int port2
)
{
unsigned int ip;
if (strToIPAddr(ip,ipString))
  {
  Target target;
  target.ip = ip;
  target.port = port;
  target.port2 = port2;
  _targetsOSC.push_back(target);
  }
}
 
void
MultiReceiver::createOSCTransmitter(
  unsigned int port
)
{
_udp = UDPTransmitter::NEW(port);
if (_udp->fail())
  {
  const StlString message = _udp->getErrorMessage();
  cerr<<"Error on upd OSC transmitter creation :"<<message<<endl;
  _udp = UDPTransmitter::nullRef();
  }
}

// forbidden methods --------------------------------------------------------
bool
MultiReceiver::_action(ArRef<Activity>, double dt)
{
if (_wantWiimote>0)
  { 
  --_wantWiimote;
  if (_wantWiimote == 0)
    {
    _wiimote = ArWiiPassive::NEW();
    if (_wiimote->connect())
      { 
      _printMessage("Wiimote connection successed",2.0);
      _wiimote->setAccMode(true);
      }
    else
      {
      _printMessage("Wiimote connection failed",2.0);
      _wiimote = ArWiiPassive::nullRef();
      }
    }
  }
if (_wiimote.valid())
  {
  _wiimote->update(); 
  if (_wiimote->pressEvent("a"))
    { 
    _printMessage("pressed [a]",1.0);
    _centerObject();
    }
  if (_wiimote->pressEvent("b"))
    { 
    _printMessage("pressed [b]",1.0);
    _catchObject();
    }
  else if (_wiimote->releaseEvent("b"))
    { 
    _printMessage("released [b]",1.0);
    _releaseObject();
    }
  else if (_wiimote->releaseEvent("1"))
    { 
    _freeze = !_freeze;
    }
  else if (_wiimote->releaseEvent("2"))
    {
    _switchObjectSkin();
    }
  if (_wiimote->getButton("up"))
    {  _tempestFollowing = true; }
  else
    { _tempestFollowing = false; }
  }
++_frame;
//text
if (_osdTimer > 0.0)
  {
  _osdTimer -= dt;
  if (_osdTimer <= 0.0)
    { 
    if (_renderer.valid())
      { _renderer->removeOSD(_osdText); }
    }
  }

//recup des sources
for (unsigned int s(0);s<_sources.size();++s)
  {
  Source & source = _sources[s];
  ArRef<UDPKinectSkeleton> receiver = source.receiver;
  //la vue trihedres est repositionnee pour toutes les sources
  for (unsigned int i(0);i<receiver->getSkeleton()->getNbJoints();++i)
    {
    ArRef<Object3D> object = source.trihedrons[i];
    object->setLocation(receiver->getSkeleton()->getJoint(i));
    source.offset->localToGlobalLocation(object);
    }
  bool visible = false;
  //cout<<"Source "<<s;
  //choix de la source utilisee en definitive == la derniere qui bouge
  StlMap<StlString,double>::iterator it = 
                            receiver->accessConfidences().find("torso");
  if (it != receiver->accessConfidences().end())
    { 
    visible = it->second > 0.5;
    if (visible && !source.visible)
      { 
      source.lastApparition = _frame;
      source.visible = true;
      if (_lastSource == -1)
        { _lastSource = s; }
      else if (_sources[_lastSource].lastApparition <= _frame)
        { _lastSource = s; }
      cout<<"Apparition source "<<s<<endl;
      }
    else if (!visible && source.visible)
      {
      source.visible = false;
      cout<<"Disparition source "<<s<<endl;
      if (_lastSource == s)
        { 
        _lastSource = -1; 
        cout<<"Plus personne"<<endl;
        } 
      }
    }

  if (_showSources)
    { _setShowSourceDebug(s,visible); }
  }

// utilisation de l'animation courante comme source
if ((_animationState == PLAY) && (_skeleton.valid()))
  {
  ArRef<KeyframeAnimation> animation = _animations[_animationSlot];
  if (animation.valid())
    {
    ArConstRef<Keyframe> keyframe = animation->evaluate( //do not interpolate
                                                _animationTimePosition,false);
    _animationTimePosition += dt;
    if (_animationTimePosition > animation->getDuration())
       { _animationTimePosition = 0.0; }
    for (unsigned int i(0);i<_skeleton->getNbJoints();++i)
      {
      _skeletonForHandle->accessJoint(i)->setLocation(keyframe->getJoint(i));
      if (!_freeze)
        { _skeleton->accessJoint(i)->setLocation(keyframe->getJoint(i)); }
      } 
    _initSkinsIfNeeded();
    }
  }
// utilisation de la source sur le skel a skins et le skel a objet
else if (_lastSource != -1)
  {
  Source & source = _sources[_lastSource];
  ArRef<Base3D> base = Base3D::NEW(); //buffer for localToGlobal
  if (_skeleton.valid())
    {
    ArRef<UDPKinectSkeleton> receiver = source.receiver;
    for (unsigned int i(0);i<receiver->getSkeleton()->getNbJoints();++i)
      {
      base->setLocation( receiver->getSkeleton()->getJoint(i) );
      source.offset->localToGlobalLocation(base);
      _skeletonForHandle->accessJoint(i)->setLocation(base);
      if (!_freeze)
        { _skeleton->accessJoint(i)->setLocation(base); }
      }
    _initSkinsIfNeeded();
    }
  }

//animations
_recordAnimationIfNeeded(dt);

//OSC targets
_OSCReEmission();

//skins
for (unsigned int i(0);i<_skins.size(); ++i)
  { _skins[i]->update(dt); }

//tempest
_tempestFollow(_skeletonForHandle->findJoint("r_hand"),dt);
_tempest->update(dt);

//calculs de deltas pour la vitesse pour la saisie d'objet
if (_objectHandle.valid())
  {
  double x,y,z;
  _objectHandle->getPosition(x,y,z);
  double roll,pitch,yaw;
  _objectHandle->extractOrientation(roll,pitch,yaw);
  Delta & delta = _deltas[_nextDelta];
  delta.x = x-_lastX;
  delta.y = y-_lastY;
  delta.z = z-_lastZ;
  delta.roll = roll - _lastRoll;
  delta.pitch = pitch - _lastPitch;
  delta.yaw = yaw - _lastYaw;
  delta.t = dt;
  _lastX = x;
  _lastY = y;
  _lastZ = z;
  _lastRoll = roll;
  _lastPitch = pitch;
  _lastYaw = yaw;
  ++_nextDelta;
  if (_nextDelta >= _nbDeltas)
    { _nextDelta = 0.0; }
  }

//quand l'objet est libre, il bouge avec les objects speed
if (!_objectHold)
  {
  ArRef<Base3D> target = _skeletonObject->findJoint("zero");
  ArRef<Base3D> handle = Base3D::NEW();
  target->attachTo(handle);
  double dRoll = _objectRotationSpeed.x()*dt;
  double dPitch = _objectRotationSpeed.y()*dt;
  double dYaw = _objectRotationSpeed.z()*dt;
  handle->setOrientation(dRoll,dPitch,dYaw);
  double dx = _objectSpeed.x()*dt;
  double dy = _objectSpeed.y()*dt;
  double dz = _objectSpeed.z()*dt;
  handle->setPosition(dx,dy,dz);
  target->attachTo(handle,Base3D::ATTACH_NONE);
  }
return true;
}

void
MultiReceiver::_printMessage(
  const StlString & message,
  double time
)
{
cout<<message<<endl;
/* TODO parameter to activate/desactivate the messages
_osdText->setText(message);
if (_renderer.valid())
  { _renderer->addOSD(_osdText); }
_osdTimer = time;
*/
}

void
MultiReceiver::_setShowSourceDebug(unsigned int number, bool debug)
{
if (number > _sources.size())
  { return; }
if (!_renderer.valid())
  { return; }
ArRef<Scene3D> scene  = _renderer->accessScene();
Source & source = _sources[number];
if (source.showDebug && !debug)
  {
  for (unsigned int i(0);i<source.trihedrons.size();++i)
    { scene->removeObject(source.trihedrons[i]); }
  }
else if (!source.showDebug && debug)
  {
  for (unsigned int i(0);i<source.trihedrons.size();++i)
    { scene->addObject(source.trihedrons[i]); }
  }
source.showDebug = debug;
}

void
MultiReceiver::_keyboardCB(const Renderer3D::KeyboardEvent & event)
{
if (!event.pressed)
  {
  cout<<"Key pressed ["<<event.key<<"]"<<endl;
  if (event.key =="a")
    { 
    _showSources = ! _showSources;
    for (unsigned int i(0);i<_sources.size();++i)
      { 
      _setShowSourceDebug(i,_showSources);
      _setSourceVolumeVisible(i,_showSources);
      }
    if (_showSources)
      { _printMessage("Show sources is ON",2.0); }
    else
      { _printMessage("Show sources is OFF",2.0); }
    }
  else if (event.key=="0")
    { _switchSkinVisible(0); }
  else if (event.key=="1")
    { _switchSkinVisible(1); }
  else if (event.key=="2")
    { _switchSkinVisible(2); }
  else if (event.key=="3")
    { _switchSkinVisible(3); }
  else if (event.key=="4")
    { _switchTempestVisible(); }
  else if (event.key=="5")
    { _switchSkinVisible(4); }
  else if (event.key=="6")
    { _switchSkinVisible(5); }
  else if (event.key=="7")
    { _switchSkinVisible(6); }
  else if (event.key=="8")
    { _switchSkinVisible(7); }
  else if (event.key=="9")
    { _switchSkinVisible(8); }
  else if (event.key==" ")
    {
    if (_objectHandle.valid())
      {
      if (_objectHold)
        { _releaseObject(); }
      else
        { _catchObject(); }
      }
    }
  else if (event.key==".")
    { _switchObjectSkin(); }
  else if (event.key=="z")
//    { _skeletonObject->findJoint("zero")->setLocation(_objectHandle); }
    { _centerObject(); }

  else if (event.key=="c")
    {
    if (!_wiimote.valid())
      {
      _wantWiimote = 10;
      _printMessage("Press 1 and 2 to connect the wiimote",3.0);
      }
    }
  else if (event.key=="f")
    { _freeze = !_freeze; }
//animations
  else if (event.key=="F1")
    { _animationControlEvent(0,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F2")
    { _animationControlEvent(1,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F3")
    { _animationControlEvent(2,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F4")
    { _animationControlEvent(3,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F5")
    { _animationControlEvent(4,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F6")
    { _animationControlEvent(5,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F7")
    { _animationControlEvent(6,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F8")
    { _animationControlEvent(7,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F9")
    { _animationControlEvent(8,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F10")
    { _animationControlEvent(9,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F11")
    { _animationControlEvent(10,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="F12")
    { _animationControlEvent(11,event.source->controlPressed(),event.source->shiftPressed()); }
  else if (event.key=="Insert")
    {
    if (event.source->controlPressed())
      { _saveAnimations(); }
    else 
      { _loadAnimations(); }
    }
  //tempest
  else if (event.key=="+")
    { 
    _tempestEmissionSpeed +=1.0; 
    _tempest->setEmissionSpeed(_tempestEmissionSpeed);
    }
  else if (event.key=="-")
    {
    _tempestEmissionSpeed =std::max(0.0,_tempestEmissionSpeed-1.0); 
    _tempest->setEmissionSpeed(_tempestEmissionSpeed);
    }
  else if (event.key=="Left")
    { _tempest->yaw(M_PI/90.0); }
  else if (event.key=="Right")
    { _tempest->yaw(-M_PI/90.0); }
  else if (event.key=="Up")
    { _tempest->pitch(-M_PI/90.0); }
  else if (event.key=="Down")
    { _tempest->pitch(M_PI/90.0); }
  else if (event.key=="Prior")
    { 
    double scale= _mobilesSkin->getSpheresScale()+0.01;
    cout<<"New sphere scale factor "<<scale<<endl;
    _mobilesSkin->setSpheresScale(scale);

    double radius = _cylindersSkin->getRadius()+0.0001;
    cout<<"New radius "<<radius<<endl;
    _cylindersSkin->setRadius(radius);

    scale = _particlesSkin->getParticlesScale()+0.001;
    cout<<"New particle scale factor "<<scale<<endl;
    _particlesSkin->setParticlesScale(scale);
    }
  else if (event.key=="Next")
    { 
    double scale= _mobilesSkin->getSpheresScale()-0.01;
    scale= std::max(0.01,scale);
    std::cout<<"New sphere scale factor "<<scale<<endl;
    _mobilesSkin->setSpheresScale(scale);

    double radius = _cylindersSkin->getRadius()-0.0001;
    radius = std::max(0.0001,radius);
    cout<<"New radius "<<radius<<endl;
    _cylindersSkin->setRadius(radius);

    scale = _particlesSkin->getParticlesScale()-0.001;
    scale= std::max(0.001,scale);
    cout<<"New particle scale factor "<<scale<<endl;
    _particlesSkin->setParticlesScale(scale);
    }
  }
}

void
MultiReceiver::_switchObjectSkin()
{
if (_objectSkin.valid())
  { _objectSkin->setVisible(!_objectSkin->getVisible()); }
}

void
MultiReceiver::_switchSkinVisible(unsigned int number)
{
if (number >= _skins.size())
  { _printMessage("No skin "+intToStr(number),2.0); }
else
  {
  _skins[number]->setVisible(!_skins[number]->getVisible());
  if (_skins[number]->getVisible())
    { _printMessage("Skin "+intToStr(number)+" visible",2.0); }
  else
    { _printMessage("Skin "+intToStr(number)+" hidden",2.0); }
  }
}

void
MultiReceiver::_setSourceVolumeVisible(unsigned int number, bool visible)
{
ArRef<Scene3D> scene = _renderer->accessScene();
if (scene.valid())
  {
  if (_sources[number].showVolume && !visible)
    { scene->removeObject(_sources[number].kinectVolume); }
  else if (!_sources[number].showVolume && visible)
    { scene->addObject(_sources[number].kinectVolume); }
  _sources[number].showVolume = visible;
  }
}

void
MultiReceiver::_computeSpeed()
{
double sx=0.0;
double sy=0.0;
double sz=0.0;
double sRoll=0.0;
double sPitch=0.0;
double sYaw=0.0;
double t=0.0;
for (unsigned int i(0);i<_nbDeltas;++i)
  {
  Delta & delta = _deltas[i];
  sx += delta.x;
  sy += delta.y;
  sz += delta.z;
  sRoll += delta.roll;
  sPitch += delta.pitch;
  sYaw += delta.yaw;
  t += delta.t;
  }
if (t!=0.0)
  { 
  _objectSpeed = Vector3d(sx,sy,sz)/t; 
  _objectRotationSpeed = Vector3d(sRoll,sPitch,sYaw)/t;
  }
}

void
MultiReceiver::_catchObject()
{
if (!_objectHandle.valid())
  { return; }
_skeletonObject->findJoint("zero")->attachTo(_objectHandle); 
_objectHold = true;
_printMessage("attrape object",2.0);
}

void
MultiReceiver::_releaseObject()
{
if (!_objectHandle.valid())
  { return; }
_skeletonObject->findJoint("zero")->attachTo(_objectHandle,Base3D::ATTACH_NONE); 
_objectHold = false;
_printMessage("lache object",2.0);
_computeSpeed();
}

void
MultiReceiver::_centerObject()
{
if (!_objectHandle.valid())
  { return; }
_skeletonObject->findJoint("zero")->setLocation(_objectHandle); 
}

void
MultiReceiver::_OSCReEmission()
{ //TODO this is a copy paste from UPDKinectSkeleton, bad
if (_udp.valid() && _skeleton.valid())
  {
  ArConstRef<Joint> torso = _skeleton->findJoint("torso");

  ArRef<MemoryBlock> memory = MemoryBlock::NEW();
  ArRef<MemoryOStream> stream = MemoryOStream::NEW(memory,0);
  _writeStringOSC("/skeleton",stream);

  ArRef<MemoryBlock> memory2 = MemoryBlock::NEW();
  ArRef<MemoryOStream> stream2 = MemoryOStream::NEW(memory2,0);
  _writeStringOSC("/skeleton_local",stream2);

  unsigned int nb = _skeleton->getNbJoints();
  StlString typeString = ",";
  for (size_t i(0);i<nb;++i)
    { typeString+="sfff"; }
  typeString+="sf";  //pour state 
  _writeStringOSC(typeString,stream);
  _writeStringOSC(typeString,stream2);

  for (unsigned int i(0);i<nb;++i)
    {
    ArConstRef<Joint> joint = _skeleton->getJoint(i);
    const StlString jointName = joint->getName();
    _writeStringOSC(jointName,stream);   
    _writeStringOSC(jointName,stream2);   
    double x,y,z;
    joint->getPosition(x,y,z);
    stream->writeFloat(x);
    stream->writeFloat(y);
    stream->writeFloat(z);

    if ( (torso.valid()) && (joint != torso) )
      { torso->globalToLocalPosition(x,y,z); }
    stream2->writeFloat(x);
    stream2->writeFloat(y);
    stream2->writeFloat(z);
    }

  //quick and dirty current state
  _writeStringOSC("state",stream);
  _writeStringOSC("state",stream2);
  if (_lastSource == -1)
    { 
    stream->writeFloat(0.0); 
    stream2->writeFloat(0.0); 
    }
  else
    { 
    stream->writeFloat(1.0); 
    stream2->writeFloat(1.0); 
    }

  for (unsigned int i(0);i<_targetsOSC.size();++i)
    {
    _udp->sendBytes(memory,0,memory->getSize(),
                                 _targetsOSC[i].ip,_targetsOSC[i].port);
    if (_targetsOSC[i].port2 != 0)
      {
      _udp->sendBytes(memory2,0,memory2->getSize(),
                                 _targetsOSC[i].ip,_targetsOSC[i].port2);
      }
    }
  }
}

void
MultiReceiver::_writeStringOSC(const StlString & value,
                                   ArRef<AbstractOStream> stream)
{ //TODO this is a copy paste from UPDKinectSkeleton, bad
unsigned int modulo = 4+4*(value.size()/4)-value.size();
for (unsigned int i(0);i<value.size();++i)
  { stream->writeChar(value[i]); }
for (unsigned int i(0);i<modulo;++i)
  { stream->writeChar(0); }
}

void
MultiReceiver::_recordAnimationIfNeeded(double dt)
{
if (_animationState != RECORD)
  { return; }
if (!_animations[_animationSlot].valid())
  { _animations[_animationSlot] = KeyframeAnimation::NEW(); }
_animations[_animationSlot]->addKeyframe(Keyframe::NEW(_skeleton),dt);
}

void
MultiReceiver::_animationControlEvent(unsigned int slot, bool recordEvent, bool eraseEvent)
{
if (slot>_animations.size())
  { 
  cerr<<"Animation slot "<<slot<<" does not exist"<<endl;
  return;
  }

if (recordEvent)
  {
  if ( (_animationState == RECORD) && (_animationSlot == slot))
    { 
    cout<<"Fin d'enregistrement, slot ["<<_animationSlot<<"]"<<endl;
    _animationState = NONE; 
    }
  else if (_animations[slot] == KeyframeAnimation::nullRef())
    {
    _animationSlot = slot;
    cout<<"Debut d'enregistrement, slot ["<<_animationSlot<<"]"<<endl;
    _animations[_animationSlot] = KeyframeAnimation::NEW(); 
    _animationState = RECORD;
    _animationTimePosition = 0.0;
    } 
  else
    { cout<<"Record event sans effet"<<endl; }
  }
else if(eraseEvent)
  {
  if (_animationState == NONE) 
    { 
    _animationSlot = slot;
    cout<<"Debut d'effacage, slot ["<<_animationSlot<<"]"<<endl;
    _animations[_animationSlot] = KeyframeAnimation::nullRef();
    cout<<"Fin d'effacage, slot ["<<_animationSlot<<"]"<<endl;
    }
  }
else //implicit play event
  {
  if ((_animationState == PLAY) && (_animationSlot == slot))
    { 
    cout<<"Stop playing, slot ["<<_animationSlot<<"]"<<endl;
    _animationState = NONE; 
    }
  else 
    {
    if (_animations[slot].valid())
      { 
      _animationSlot = slot;
      cout<<"Start playing, slot ["<<_animationSlot<<"]"<<endl;
      _animationState = PLAY;
      _animationTimePosition = 0.0;
      } 
    else
      { _animationState = NONE; }
    }
  }
}

void
MultiReceiver::_loadAnimations()
{
_animationState = NONE;
for (size_t i(0);i<_animations.size();++i)
  {
  StlString filename = "animation_slot"+uintToStr(i)+".bvh";
  _loadAnimation(i,filename);
  }
}

void
MultiReceiver::_saveAnimations()
{
for (size_t i(0);i<_animations.size();++i)
  {
  if (_animations[i].valid())
    {
    StlString filename = "animation_slot"+uintToStr(i)+".bvh";
    _saveAnimation(_animations[i],filename);
    }
  }
}

void 
MultiReceiver::_saveAnimation(
  ArRef<KeyframeAnimation> animation,
  const StlString & name
)
{
if (animation.valid())
  { 
  cout<<"Debut export de l'animation ["<<name<<"]"<<endl;
  animation->exportBVH(name);
  cout<<"Fin export de l'animation ["<<name<<"]"<<endl;
  }
}

void
MultiReceiver::_loadAnimation(size_t slot,const StlString & name )
{
ifstream input(name.c_str());
if (!input)
  { 
  cerr<<"Pas de fichier d'animation ["<<name<<"]"<<endl;
  return;
  }
input.close();

LoaderData data = HLibLoader::loadFile(name,false); //no cache
if (data.failed)
  { 
  cerr<<"Failed loading animation ["<<name<<"]"<<endl; 
  return;
  }
data.animation->newOrder(_skeleton);
_animations[slot] = data.animation;
}

void
MultiReceiver::_initTempest()
{
_tempestX = -5.0;
_tempestEmissionSpeed = 40.0;
_tempestTexture = URLTexture::NEW("data/rond.png",false,false);
_renderer->accessScene()->addParticleSystem(_tempest);
_tempest->setLocation(_renderer);
_tempest->translate(_tempestX,0.0,0.0);
_tempest->setSection(25.0,25.0,0.0,0.0,true); //rounded
_tempest->setEmissionSpeed(_tempestEmissionSpeed);
_tempest->setEmissionSpeedDispersion(0.5);
_tempest->setInitialAngleDispersion(0.1);
_tempest->setParticleDuration(5.0);
_tempest->setLighting(false);
_tempest->setSize(0.04,0.04);
_tempest->setSizeDispersion(0.01,0.01);
_tempest->setColor(1.0,1.0,1.0,1.0);
_tempest->setTexture(_tempestTexture);
_tempest->setParticleRate(400.0);
_tempest->setGravity(0.0,0.0,0.0);

}

void
MultiReceiver::_switchTempestVisible()
{
assert(_renderer.valid());
if (_tempestVisible)
  { 
  _renderer->accessScene()->removeParticleSystem(_tempest); 
  _tempestVisible = false;
  }
else
  { 
  _renderer->accessScene()->addParticleSystem(_tempest); 
  _tempestVisible = true;
  }
}

void
MultiReceiver::_tempestFollow(ArConstRef<Base3D> target, double dt)
{
if (!target.valid())
  { return; }
if (!_renderer.valid())
  { return; }
double x,y,z;
target->getPosition(x,y,z);
_renderer->globalToLocalPosition(x,y,z);
_tempest->setLocation(_renderer);
if (_tempestFollowing)
  {
  _tempestYaw = atan2(y,x);
  _tempestPitch = -atan2(z,sqrt(x*x+y*y));
  }
else
  {
  if (ArSystem::realRand()*_tempestChangeDelay<0.01)
     {
     const double angle = M_PI/4; //45deg
     _tempestYaw = ArSystem::realRand()*angle -  0.5*angle;
     _tempestPitch = ArSystem::realRand()*angle -  0.5*angle;
     }
  }
  _tempest->pitch(_tempestPitch);
  _tempest->yaw(_tempestYaw);
  _tempest->translate(_tempestX,0.0,0.0);
}

//configuration parameters --------------------------------------------------
struct Configuration {
  enum Mode { SOURCE, TARGET, UNDEFINED };
  Mode mode;

  struct Address {
    StlString ip;
    int port;
    bool OSC; //emission of positions only, via one OSC message
    int port2; //alternative version, local to torso, only if OSC true
  };

  int kinect; //0 autodetect, 1 device one, 2 device two ...
  int sourcePort; //need a port to create the udp transmitter, source mode

  struct Source {
    int receptionPort;
    ArRef<Base3D> offset;
  };

  //sources are used in target mode
  StlVector<Source> sources;
  //targets are used in source mode, and in target mode for OSC re-emission
  StlVector<Address> targets;
};

void
readBase3D(ArRef<XmlNode> baseNode, ArRef<Base3D> target)
{
if (!target.valid())
  { return; }
ArRef<Base3D> base = Base3D::NEW();
ArRef<XmlNode> node = baseNode->getFirstChild();
while (node.valid())
  {
  const StlString name(node->getName());
  if (name=="Translate")
    { 
    double x=0.0;
    node->getPropertyReal("x",x);
    double y=0.0;
    node->getPropertyReal("y",y);
    double z=0.0;
    node->getPropertyReal("z",z); 
    base->translate(x,y,z);
    }
  else if (name=="Roll")
    {
    double value=0.0;
    node->getPropertyReal("value",value);
    base->roll(M_PI*value/180.0);
    }
  else if (name=="Pitch")
    {
    double value=0.0;
    node->getPropertyReal("value",value);
    base->pitch(M_PI*value/180.0);
    }

  else if (name=="Yaw")
    {
    double value=0.0;
    node->getPropertyReal("value",value);
    base->yaw(M_PI*value/180.0);
    }
  node = node->getNext();
  }
target->setLocation(base);
}

Configuration readConfigurationFile(StlString filename)
{
Configuration configuration;
configuration.mode = Configuration::UNDEFINED;

ArRef<XmlParser> parser = XmlParser::NEW();
if (!parser->parseFile(filename))
  {
  cerr<<"Error parsing "<<filename<<endl;
  return configuration;
  }

ArRef<XmlNode> root = parser->getRoot();

configuration.sourcePort=9800;
configuration.kinect=0; //autodetect
StlString mode="source";
root->getPropertyString("mode",mode);
if (mode == "source")
  { 
  configuration.mode = Configuration::SOURCE; 
  root->getPropertyInteger("port",configuration.sourcePort);
  root->getPropertyInteger("kinect",configuration.kinect);
  }
else if (mode == "target")
  { 
  configuration.mode = Configuration::TARGET; 
  //using it for OSC re emission in target mode
  root->getPropertyInteger("port",configuration.sourcePort);
  }
else //undefined
  { return configuration; }

ArRef<XmlNode> child = root->getFirstChild();
while (child.valid())
  {
  if (child->getName() == "Target")
    {
    Configuration::Address address;
    address.ip="";
    child->getPropertyString("ip",address.ip);
    address.port=9876;
    child->getPropertyInteger("port",address.port);
    address.OSC=false;
    child->getPropertyBoolean("OSC",address.OSC);
    address.port2=0;
    child->getPropertyInteger("port2",address.port2);
    configuration.targets.push_back(address);
    }
  else if (child->getName() == "Source")
    {
    Configuration::Source source;
    source.receptionPort=9801;
    child->getPropertyInteger("port",source.receptionPort);
    source.offset=Base3D::NEW();
    readBase3D(child,source.offset);
    configuration.sources.push_back(source);
    }
  child = child->getNext();
  }
return configuration;
}

//main ----------------------------------------------------------------------

//

ArRef<Scheduler> simulationInit(void)
{
ArRef<Scheduler> scd = RealTimeScheduler::NEW(1e-3);

if (ArSystem::getCommandLine().size()<2)
  {
  cerr<<"Usage :"<<ArSystem::getCommandLine()[0]
                                      <<" configurationFile.xml"<<endl;
  exit(-1);
  }

Configuration configuration = 
                 readConfigurationFile(ArSystem::getCommandLine()[1]);

if (configuration.mode==Configuration::UNDEFINED)
  {
  cout<<"Configuration file invalid"<<endl;
  exit(-1);
  }

if (configuration.mode==Configuration::SOURCE)
  {
  ArRef<UDPKinectSkeleton> emitter = UDPKinectSkeleton::NEW();
  for (unsigned int i(0);i<configuration.targets.size();++i)
    {
    emitter->addTarget(configuration.targets[i].ip,
                       configuration.targets[i].port,
                       configuration.targets[i].OSC,
                       configuration.targets[i].port2);
    }
  emitter->setTransient(false); //Ugly
  
  emitter->initEmitKinect(configuration.sourcePort,configuration.kinect);
  cout<<"Source mode"<<endl;
  }
else
  { 
  ArRef<Viewer3D> viewer=Viewer3D::NEW();
  viewer->setTransient(false); //Ugly
  ArRef<SimpleInteractor> interactor = SimpleInteractor::NEW();
  interactor->setTransient(false); //Ugly
  interactor->setRenderer(viewer);
  viewer->setCloseAction(Window3D::CLOSE_LEAVE);
  
  ArRef<Scene3D> scene = Scene3D::NEW();
  viewer->selectScene(scene);

  unsigned int w,h;
  Renderer3D::getScreenSize(w,h);
  viewer->setWindowGeometry(w-2048,0,2048,1080);
  viewer->setMapped(true);
  viewer->setPosition(-2.0,0.0,0.0);
  const double far = 1000.0;
  viewer->setFarDistance(far);
  viewer->setNearDistance(0.0001*far);
 // viewer->setHeadLightColor(0.2,0.1,0.0);
  viewer->setBackgroundColor(0.0,0.0,0.0);

  ArRef<Light3D> light = Light3D::NEW();
  scene->addLight(light);
  light->setColor(1.0,1.0,1.0);
  light->yaw(M_PI/4.0);
  light->pitch(M_PI/4.0);
  light->setPosition(-5.0,20.0,5.0);
  light->setTransient(false);

  ArRef<MultiReceiver> multiReceiver = MultiReceiver::NEW();
  multiReceiver->setTransient(false); //Ugly
  multiReceiver->setRenderer(viewer);
  
  for (unsigned int i(0);i<configuration.sources.size();++i)
    {
    cout<<"Source "<<i<<endl;
    ArRef<UDPKinectSkeleton> receiver = UDPKinectSkeleton::NEW();
    Configuration::Source & source = configuration.sources[i];
    receiver->setReceptionPort(source.receptionPort);
    receiver->initReceiveKinect();
    multiReceiver->addSource(receiver,source.offset);
    }

  for (unsigned int i(0);i<configuration.targets.size();++i)
    {
    Configuration::Address & address = configuration.targets[i];
    multiReceiver->addOSCTarget(address.ip,address.port,address.port2);
    }
  multiReceiver->createOSCTransmitter(configuration.sourcePort);
  cout<<"Target mode"<<endl; 
  }
  
return scd;
}

void receive(){
	sockaddr_in si_me, si_other;
	int s;
	assert((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))!=-1);
	int port=7400;
	int broadcast=1;

	setsockopt(s, SOL_SOCKET, SO_BROADCAST,	&broadcast, sizeof broadcast);

	memset(&si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port);
	si_me.sin_addr.s_addr = INADDR_ANY;
	
	assert(bind(s, (sockaddr *)&si_me, sizeof(sockaddr))!=-1);

	char buf[12];
	unsigned slen=sizeof(sockaddr);
	int nb_char = recvfrom(s, buf, sizeof(buf), MSG_WAITALL, (sockaddr *)&si_other, &slen);
	
	if (nb_char == 12)
	{
		char * end = buf + nb_char - 1;
		unsigned char result_char = (unsigned char)*end;
		unsigned int result_int = 0x00FF&result;

		cout<<"recv: "<<result_int<<endl;
	}
	else
	{
		cout<<"ERROR IN RECEPTION"<<endl;
	}
}

void *commandReceiver(void*){
  cout<<"Thread cree"<<endl;
	
	receive();
	
	cout<<"Thread termine"<<endl;
	return NULL;
}

int main(int argc, char** argv)
{
ArSystem arevi(argc, argv);

MultiReceiver::REGISTER_CLASS();
hLibInit(); //registers hlib's classes

ArSystem::loadPlugin("Imlib2ImageLoader");
ArSystem::loadPlugin("MagickImageLoader");
//ArSystem::loadPlugin("OSXImageLoader");
//ArSystem::loadPlugin("CairoTexture");
ArSystem::loadPlugin("XmlParser");

//création du thread de commande distante
pthread_t task_command;
if(pthread_create(&task_command, NULL, commandReceiver, NULL)) {
	fprintf(stderr, "Error creating command receiver thread\n");
	return 1;
}

KinectOpenNI2::initNiTE();
ArSystem::simulationLoop(&simulationInit);
KinectOpenNI2::shutdownNiTE();

return 0;
}
