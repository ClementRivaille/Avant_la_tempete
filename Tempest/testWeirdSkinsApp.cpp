#include <AReVi/arClass.h>

#include "app.h"
#include "weirdSkins.h"

using std::cerr;
using std::cout;
using std::endl;

AR_CLASS_DEF(App,Viewer3D)


App::App(ArCW& arCW)
  : Viewer3D(arCW),
    _interactor(SimpleInteractor::NEW())
{
  _interactor->setObjectInteraction(true);
  _interactor->setRenderer(thisRef());

  assert(ArSystem::getCommandLine().size() >= 2);

  ArRef<Scene3D> _scene=Scene3D::NEW();  

  StlString str =  ArSystem::getCommandLine()[1];
  cerr << "Loading " << str << " ...\t";
  LoaderData data = HLibLoader::loadFile(str,true); //use cache
  if(data.failed) {
    cerr << "FAILED" << endl;
  } else {
    cerr << "done" << endl;
  }
  _body = data.body;
  if (data.body.valid())
    {
    _scene->addObject(_body);   
    _skeleton = Skeleton::NEW(_body->getBodyShape()->getSkeleton());
    }
  else
    {
    cout<<"No body"<<endl;
    _skeleton = Skeleton::NEW();
    }


  if(data.animation != Animation::nullRef()) {    
    _animation = data.animation;
    _body->applyPose(_animation->getKeyframe(0));
  }

  if(ArSystem::getCommandLine().size() > 2)
    {
    str =  ArSystem::getCommandLine()[2];
    cerr << "Loading _animation " << str << " ...\t" << endl;
    _animation = HLibLoader::loadAnimation(str,true,_skeleton); //use cache
    if(_animation != Animation::nullRef()) 
      {
      ArConstRef<Keyframe> keyframe = _animation->getKeyframe(0);
      _body->applyPose(keyframe);
      for (unsigned int i(0);i<_skeleton->getNbJoints();++i)
        { _skeleton->accessJoint(i)->setLocation(keyframe->getJoint(i)); }
      }
    }

  //_body->accessBodyShape()->setShowSkeleton(false);
  

  ArRef<BoundingBox3D> bbox = BoundingBox3D::NEW();
  _body->accessShape()->readBoundingBox(bbox);
  setPosition(-2 *  bbox->getMaxSize(),0,bbox->getMaxSize() / 2.0);
  setFarDistance(50.0*bbox->getMaxSize());
  selectScene(_scene);
  ArRef<Activity> act = Activity::NEW(0.04);
  act->setBehavior(thisRef(), &App::_action);

  _skins.push_back(TrihedronSkin::NEW());
  ArRef<SpheresSkin> spheresSkin = SpheresSkin::NEW();
  spheresSkin->setParameters(0.5,3.0,40,0.001,0.025);
  _skins.push_back(spheresSkin);
  spheresSkin = SpheresSkin::NEW();
  spheresSkin->setParameters(0.5,0.1,5,0.05,0.15);
  _skins.push_back(spheresSkin);
  _initSkins();
}

App::~App()
{
}

void
App::_initSkins()
{
for (unsigned int i(0);i<_skins.size();++i)
  {
  ArRef<Skin0> skin0 = _skins[i];
  if (skin0->createGeometries(_skeleton))
    { skin0->setVisible(accessScene(),true); }
  else
    { cerr<<skin0->getErrorMessage()<<endl; }
  }
}

bool App::_action(ArRef<Activity> act, double /*dt*/)
{
  if(_animation != Animation::nullRef()) {
    ArConstRef<Keyframe> keyframe = _animation->evaluate(fmod((double)act->getTime(),(double)_animation->getDuration()));
    _body->applyPose(keyframe);
    for (unsigned int i(0);i<_skeleton->getNbJoints();++i)
      { _skeleton->accessJoint(i)->setLocation(keyframe->getJoint(i)); }
    return true;
  }
  else {
    return false;
  }
}

ArRef<Viewer3D> viewer;

ArRef<Scheduler> simulationInit(void) {

  if(ArSystem::getCommandLine().size() < 2) {
    cerr << "usage : " << ArSystem::getCommandLine()[0];
    cerr << " skeleton_file _animation_file (optional)" << endl;
    return(Scheduler::nullRef());
  }

  ArRef<Scheduler> scd = RealTimeScheduler::NEW(1e-3);

  viewer=App::NEW();
  viewer->setCloseAction(Window3D::CLOSE_LEAVE);

  unsigned int w,h;
  Renderer3D::getScreenSize(w,h);
  viewer->setWindowGeometry(w-800,0,800,800);
  viewer->setMapped(true);
  viewer->setBackgroundColor(0.0,0.0,0.0);

  return scd;
}


int main(int argc, char** argv) {
  ArSystem arevi(argc, argv);

  hLibInit();
  App::REGISTER_CLASS();
  Skin0::REGISTER_CLASS(); //ancestor in weid skins
  TrihedronSkin::REGISTER_CLASS();
  SpheresSkin::REGISTER_CLASS();

  ArSystem::loadPlugin("Imlib2ImageLoader");
  ArSystem::loadPlugin("MagickImageLoader");

  ArSystem::simulationLoop(&simulationInit);

  return 0;
}
