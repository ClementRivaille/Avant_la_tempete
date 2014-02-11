#include "AReVi/arSystem.h"

using namespace AReVi;
using namespace std;

#include "AReVi/activity.h"
#include "AReVi/Lib3D/viewer3D.h"
#include "AReVi/Lib3D/simpleInteractor.h"
#include "AReVi/Lib3D/object3D.h"
#include "AReVi/Lib3D/transform3D.h"
#include "AReVi/Lib3D/base3D.h"
#include "AReVi/Lib3D/scene3D.h"

#include "AReVi/Shapes/shape3D.h"
#include "AReVi/Shapes/pointSet3D.h"

#include "AReVi/Utils/fileStream.h"

#include "libfreenect/libfreenect.h"
#include "libfreenect/libfreenect_sync.h"

#include "AReVi/Contrib/arMath.h"


//#include "AReVi/Lib3D/particleSystem.h"
#include "AReVi/Lib3D/urlTexture.h"


//ajout du nouveau systeme de particules
#include "./particlesEtForces.h"


using std::cout;
using std::endl;
using std::cerr;
//----------------------------------------------------------------------------
// algos pour skeletiser
//----------------------------------------------------------------------------
struct EuclideanMetric
{
  static inline int f (int x_i, int gi)
  {
    return (x_i*x_i)+gi*gi;
  }

  static inline int sep (int i, int u, int gi, int gu, int) //noexcept
  {
    return (u*u - i*i + gu*gu - gi*gi) / (2*(u-i));
  }
};

//a priori ne marche po
struct ManhattanMetric
{
  static inline int f (int x_i, int gi)
  {
    return abs (x_i) + gi;
  }

  static inline int sep (int i, int u, int gi, int gu, int inf) //noexcept
  {
    if (gu >= gi + u - i)
      return inf;
    else if (gi > gu + u - i)
      return -inf;
    else
      return (gu - gi + u + i) / 2;
  }
};

//a priori ne marche po
struct ChessMetric
{
  static inline int f (int x_i, int gi)
  {
    return std::max (abs (x_i), gi);
  }

  static inline int sep (int i, int u, int gi, int gu, int) //noexcept
  {
    if (gi < gu)
      return std::max (i+gu, (i+u)/2);
    else
      return std::min (u-gi, (i+u)/2);
  }
};


template <class Metric>
static void calculate (std::vector<int> & f, std::vector<bool> test, int const m, int const n, Metric metric)
{
  std::vector <int> g (m * n);

  int const inf = m + n;

  // phase 1
  {
    for (int x = 0; x < m; ++x)
    {
      //g [x] = test (x, 0) ? 0 : inf;
      g [x] = test[x+0*m] ? 0 : inf;

      // scan 1
      for (int y = 1; y < n; ++y)
      {
        int const ym = y*m;
        //g [x+ym] = test (x, y) ? 0 : 1 + g [x+ym-m];
        g [x+ym] = test[x+y*m] ? 0 : 1 + g [x+ym-m];
      }

      // scan 2
      for (int y = n-2; y >=0; --y)
      {
        int const ym = y*m;

        if (g [x+ym+m] < g [x+ym])
          g [x+ym] = 1 + g[x+ym+m];
      }
    }
  }

  // phase 2
  {
    std::vector <int> s (std::max (m, n));
    std::vector <int> t (std::max (m, n));

    for (int y = 0; y < n; ++y)
    {
      int q = 0;
      s [0] = 0;
      t [0] = 0;

      int const ym = y*m;

      // scan 3
      for (int u = 1; u < m; ++u)
      {
        while (q >= 0 && metric.f (t[q]-s[q], g[s[q]+ym]) > metric.f (t[q]-u, g[u+ym]))
          q--;

        if (q < 0)
        {
          q = 0;
          s [0] = u;
        }
        else
        {
          int const w = 1 + metric.sep (s[q], u, g[s[q]+ym], g[u+ym], inf);

          if (w < m)
          {
            ++q;
            s[q] = u;
            t[q] = w;
          }
        }
      }

      // scan 4
      for (int u = m-1; u >= 0; --u)
      {
        int const d = metric.f (u-s[q], g[s[q]+ym]);
        //double dd = d;
        //double root = sqrt(dd);
        //f [u+y*m] = int(root);
        f [u+y*m] = d;
        if (u == t[q])
          --q;
      }
    }
  }
}

static
void
localMaximums4(std::vector<int> & result, const std::vector<int> & source, const int m, const int n)
{
for (int y = 1; y<(n-1); ++y)
  {
  for (int x = 1; x<(m-1); ++x)
    {
    const int p = y*m+x;
    const int p0 = p-m;
    const int p1 = p-1;
    const int p2 = p+1;
    const int p3 = p+m;
    const int v = source[p];
    if ( (source[p0] <= v) && (source[p1] <= v) && (source [p2] <= v) && (source[p3]<=v))
      { result[p] = v; }
    else
      { result[p] = 0; }
    }
  }
}

static
void
localMaximums8(std::vector<int> & result, const std::vector<int> & source, const int m, const int n)
{
for (int y = 1; y<(n-1); ++y)
  {
  for (int x = 1; x<(m-1); ++x)
    {
    const int p = y*m+x;
    const int p0 = p-m-1;
    const int p1 = p-m;
    const int p2 = p-m+1;
    const int p3 = p-1;
    const int p4 = p+1;
    const int p5 = p+m-1;
    const int p6 = p+m;
    const int p7 = p+m+1;
    const int v = source[p];
    if (    (source[p0] <= v) && (source[p1] <= v) && (source [p2] <= v) && (source[p3]<=v)
         && (source[p4] <= v) && (source[p5] <= v) && (source [p6] <= v) && (source[p7]<=v) )
      { result[p] = v; }
    else
      { result[p] = 0; }
    }
  }
}


static
void
sommeDelta8(std::vector<int> & result, const std::vector<int> & source, const int m, const int n)
{
for (int y = 1; y<(n-1); ++y)
  {
  for (int x = 1; x<(m-1); ++x)
    {
    const int p = y*m+x;
    const int p0 = p-m-1;
    const int p1 = p-m;
    const int p2 = p-m+1;
    const int p3 = p-1;
    const int p4 = p+1;
    const int p5 = p+m-1;
    const int p6 = p+m;
    const int p7 = p+m+1;
    const int v = source[p];
    int r = 0;
    r += v - source[p0];
    r += v - source[p1];
    r += v - source[p2];
    r += v - source[p3];
    r += v - source[p4];
    r += v - source[p5];
    r += v - source[p6];
    r += v - source[p7];
    result[p] = r;
    }
  }
}

static
void
printMatrix(std::vector<int> & data, const int m, const int n, bool doNotShowZero = false)
{
for (int y(0);y<n;++y)
  {
  for (int x(0);x<m;++x)
    {
    int d = data[y*m+x];
    if (d==0)
      { 
      if (doNotShowZero)
        { std::cout<<"   "; }
      else
        { std::cout<<d<<"  "; }
      }
    else if (d>9)
      { std::cout<<d<<" "; }
    else
      { std::cout<<d<<"  "; }
    }
  std::cout<<std::endl;
  }
}

//----------------------------------------------------------------------------
class Action : public ArObject
{
public:
  AR_CLASS(Action)
  AR_CONSTRUCTOR(Action)

  virtual
  void
  keyboardCB(const Renderer3D::KeyboardEvent & evt);

  virtual
  void
  setRenderer(ArRef<Renderer3D> renderer);

protected:
  virtual
  void
  _computeColorCoordinatesForPoint(const Vector3d & point3D, int & u, int & v);

  virtual
  bool
  _action(ArRef<Activity>, double);

  virtual
  void
  _loadBackground();
  
  //initialisation de la tempete
  virtual
  void
  _initTempest();
  
  virtual
  void
  _tempestFollow(double dt);

protected:
  ArRef<Object3D> _obj;
  ArRef<PointSet3D> _points;
  ArRef<Transform3D> _transform;

  int _depthLimit;
  StlVector<short> _lastFrame;
  StlVector<short> _fond;
  int _thickness;

  ArRef<Base3D> _startLocation;
  ArPtr<Renderer3D> _renderer;
  
  
  //ajout d'un systeme de particules, similaire a tempest dans app.cpp
  //ArRef<ParticleSystem> _tempest;
  ArRef<ParticlesEtForces> _tempest;
  ArRef<URLTexture> _tempestTexture;
  double _tempestEmissionSpeed;
  bool _tempestVisible;
  double _tempestX;
  bool _tempestFollowing;
  double _tempestYaw;
  double _tempestPitch;
  double _tempestChangeDelay;

  
};

AR_CLASS_DEF(Action,ArObject)

Action::Action( ArCW & arCW)
  : ArObject(arCW),
  _obj(Object3D::NEW()),
  _points(PointSet3D::NEW()),
  _transform(Transform3D::NEW()),
  _depthLimit(1000),
  _lastFrame(640*480,1000),
  _fond(640*480,1000),
  _thickness(7),
  _startLocation(Base3D::NEW()),
  //ajout en rapport avec les particules
  //_tempest(ParticleSystem::NEW()),
  _tempest(ParticlesEtForces::NEW()),
  _tempestVisible(true),
  _tempestFollowing(false),
  _tempestYaw(0.0),
  _tempestPitch(0.0),
  _tempestChangeDelay(0.01)
{
_startLocation->setPosition(1.9364, 5.73606, -0.117783);
_startLocation->setOrientation(-0.173945,0.00944314, -0.342706);
ArRef<Activity> activity = Activity::NEW(1.0/30.0);
activity->setBehavior(thisRef(),&Action::_action);

ArRef<Shape3D> shape = Shape3D::NEW();
_obj->setShape(shape);

_points->setDisplayListUsage(false);
shape->addRootPart(_points);

//mesh transformation matrix from lib freenect glpclview example
/*
double * mat = _transform->accessTransformation().matrix;

const double fx = 594.21f;
const double fy = 591.04f;
const double a = -0.0030711f;
const double b = 3.3309495f;
const double cx = 339.5f;
const double cy = 242.7f;

//mat[Util3D::R00] = 1/fx;
//mat[Util3D::R11] = -1/fy;
//mat[Util3D::Z2] = a;
//mat[Util3D::TX] = -cx/fx;
//mat[Util3D::TY] = cy/fy;
//mat[Util3D::TZ] = -1;
//mat[Util3D::ONE] = b;
*/

/*
GLfloat mat[16] = {
    1/fx,       0,  0, 0,
    0,      -1/fy,  0, 0,
    0,          0,  0, a,
    -cx/fx, cy/fy, -1, b
};
*/

_transform->preTranslate(0.0,0.0,0.0); //to take into account the modification of mat
_points->writeTransformation(_transform);

_points->setThickness(_thickness);

_loadBackground();
}

Action::~Action()
{
}

void
Action::setRenderer(ArRef<Renderer3D> renderer)
{
_renderer = renderer;
if (_renderer.valid())
  {
	_renderer->setLocation(_startLocation);
	
	//initialisation du systeme de particules
	_initTempest();
  }
}

void
Action::_computeColorCoordinatesForPoint(const Vector3d & point3D, int & u, int & v)
{
const Matrix3x3d matrix ( 
  9.9984628826577793e-01, 1.2635359098409581e-03, -1.7487233004436643e-02, 
 -1.4779096108364480e-03, 9.9992385683542895e-01, -1.2251380107679535e-02,
  1.7470421412464927e-02, 1.2275341476520762e-02, 9.9977202419716948e-01 );

/*
const Matrix3x3d matrix ( 
  9.9984628826577793e-01, -1.4779096108364480e-03, 1.7470421412464927e-02, 
  1.2635359098409581e-03, 9.9992385683542895e-01, 1.2275341476520762e-02, 
  -1.7487233004436643e-02, -1.2251380107679535e-02, 9.9977202419716948e-01 );
*/

const Vector3d translation (1.9985242312092553e-02, -7.4423738761617583e-04, -1.0916736334336222e-02 );
Vector3d p = matrix * point3D + translation;
const double fx_rgb = 5.2921508098293293e+02;
const double fy_rgb = 5.2556393630057437e+02;
const double cx_rgb = 3.2894272028759258e+02;
const double cy_rgb = 2.6748068171871557e+02;

u = (p.x() * fx_rgb / p.z() ) + cx_rgb;
if (u<0) 
  { u=0; }
else if (u>=640)
  { u=639; }
v = (p.y() * fy_rgb / p.z() ) + cy_rgb;
if (v<0)
  { v=0; }
else if (v>=480)
  { v=479; }
}

bool
Action::_action(ArRef<Activity>, double dt)
{
	
	
	
const double a_d = -0.0030711;
const double b_d = 3.3309495;
const double fx = 594.21;
const double fy = 591.04;
const double cx = 339.5;
const double cy = 242.7;

short * depth = 0;
uint32_t ts;
if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
  { exit(-1); }

unsigned char * rgb= 0;
if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
  { exit(-1); }

StlVector<Util3D::Dbl3> & vertices = _points->accessVertices();
StlVector<Util3D::Dbl3> & colors = _points->accessColors();

const int m = 640;
const int n = 480;
const size_t dataSize = m*n;
vector<bool> rawBool(dataSize,true);
for (size_t i(0);i<dataSize;++i)
  {
  if (depth[i]<_depthLimit)
    { rawBool[i] = false; }
  }
vector<int> distances(dataSize,0);
EuclideanMetric metric;
calculate(distances,rawBool,m,n,metric);
std::vector<int> medialAxisTransform(dataSize);
localMaximums8(medialAxisTransform,distances,m,n);   
std::vector<int> sommeDelta(dataSize,0);
sommeDelta8(sommeDelta,distances,m,n);

if (vertices.empty()) //first time, memory allocation
  {
  Util3D::Dbl3 vertex(0.0,0.0,0.0);
  vertices.resize(640*480,vertex);
  colors.resize(640*480,vertex);
  }
const Util3D::Dbl3 white(1.0,1.0,1.0);
const Util3D::Dbl3 gray(0.5,0.5,0.5);
const Util3D::Dbl3 dark(0.2,0.2,0.2);
const Util3D::Dbl3 blue(0.0,0.0,0.5);
for (int i(0);i<480;++i)
  {
  for (int j(0);j<640;++j)
    {
    const int offset = i*640+j;
    const short rawDepth = depth[offset];
    _lastFrame[offset] = rawDepth;
    double depth = 0.0;
    if ( rawDepth < _fond[offset] )
      {
      depth = 2.0 / ( rawDepth * a_d + b_d );
      }

    double x = (j - cx) * depth / fx;
    double y = (i - cy) * depth / fy;
    bool keep = medialAxisTransform[offset] > 15;
    keep = keep || (sommeDelta[offset] > 50);
    if (keep)
      { x +=5.0; }

    if (!keep)
      { depth = 0.0; }
    vertices[i*640+j]=Util3D::Dbl3(depth,x,-y);
    
    
    
    
    
    
    
    int u,v;
    _computeColorCoordinatesForPoint(Vector3d(x,y,depth),u,v);

    const unsigned int colorOffset = (v*640+u)*3;
    const double r = rgb[colorOffset]/256.0;
    const double g = rgb[colorOffset+1]/256.0;
    const double b = rgb[colorOffset+2]/256.0;
    colors[i*640+j]=Util3D::Dbl3(r,g,b);
    
    
    
    
    
    
    if (keep)
      {  
      const double v =  0.005 * 
           double(std::max(medialAxisTransform[offset],sommeDelta[offset]));
      colors[offset]=Util3D::Dbl3(v,v,v);
      }
    else if (medialAxisTransform[offset] != 0)
      {  colors[offset]=blue; }
    else if (distances[offset] != 0)
      { 
      const double v = sqrt(double(distances[offset]))*0.01;
      colors[offset]= Util3D::Dbl3(v,v,v); 
      }
    else
      { colors[offset]=dark; }
    }
  }

if (!_points->applyChanges(true))
  { cerr<<"Invalid point set"<<endl; }
  
  
  
  
  
//tempete
//_tempestFollow(dt);
_tempest->update(dt);
  
return true;
}

void
Action::_loadBackground()
{
cout<<"Chargement du fond, debut"<<endl;
ArRef<FileIStream> fileStream = FileIStream::NEW("sauvegardeFond.bin");
StlVector<short> tmp(_fond.size(),_depthLimit);
bool ok = true;
for (size_t i(0); i<_fond.size(); ++i)
  { 
  if ( !(fileStream->readShort(tmp[i])) )
    { 
    ok = false;
    break;
    }
  }
if (ok)
  { 
  for (size_t i(0); i<_fond.size(); ++i)
    { _fond[i] = tmp[i]; }
  cout<<"Chargement du fond terminee"<<endl;
  }
else
  { cerr<<"Erreur pendant le chargement du fond"<<endl; }
}


void
Action::keyboardCB(const Renderer3D::KeyboardEvent & event)
{
if (!event.pressed)
  {
  cout<<"Key pressed ["<<event.key<<"]"<<endl;
  if (event.key=="F1")
    {
    cout<<"reset du fond"<<endl;
    for (size_t i(0); i<640*480; ++i)
      { _fond[i] = _depthLimit; }
    }
  else if (event.key=="F2")
    { 
    cout<<"accumulation du fond"<<endl;
    for (size_t i(0); i<640*480; ++i)
      { 
      short int frame = _lastFrame[i]-4;
      _fond[i] = std::min(_fond[i],frame); 
      }
    } 
  else if (event.key=="F3")
    {
    cout<<"Sauvegarde du fond, debut"<<endl;
    ArRef<FileOStream> fileStream = FileOStream::NEW("sauvegardeFond.bin",false);
    bool ok = true;
    for (size_t i(0); i<_fond.size(); ++i)
      { 
      if ( !(fileStream->writeShort(_fond[i])) )
        { 
        ok = false;
        break;
        }
      }
    if (ok)
      { cout<<"Sauvegarde du fond terminee"<<endl; }
    else
      { cerr<<"Erreur pendant la sauvegarde du fond"<<endl; }
    }
  else if (event.key=="F4")
    { _loadBackground(); }
  else if (event.key=="Next")
    {
    _thickness = std::max(1,_thickness-1);
    _points->setThickness(_thickness);
    }
  else if (event.key=="Prior")
    {
    _thickness +=1;
    _points->setThickness(_thickness);
    }
  else if (event.key==" ")
    {
    if (_renderer.valid())
      {
      double x,y,z;
      _renderer->getPosition(x,y,z);
      double roll,pitch,yaw;
      _renderer->extractOrientation(roll,pitch,yaw); 
      cout<<"Current location was "<<x<<", "<<y<<", "<<z<<" orientation : "<<roll<<","<<pitch<<", "<<yaw<<endl;
      cout<<"Return to start location"<<endl;
      _renderer->setLocation(_startLocation);
      }
    }
  }
}

//systeme de particules
void Action::_initTempest()
{
	/*_renderer->accessScene()->addParticleSystem(_tempest);
	_tempest->setLocation(_startLocation);
    _tempest->setGravity(0.0,0.0,0.0);*/
    
    _tempestX = -5.0;
	_tempestEmissionSpeed = 4.0;
	//_tempestTexture = URLTexture::NEW("data/rond.png",false,false);
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
	//_tempest->setTexture(_tempestTexture);
	_tempest->setParticleRate(400.0);
	_tempest->setGravity(0.0,0.0,0.0);
	
	//construction des potentiels et champs
	ArRef<Attracteur> attrac = Attracteur::NEW();
	attrac->setOrigin(Util3D::Dbl3(-6.0,3.0,2.0));
	attrac->setForce(0.1);
	
	_tempest->m_potentiels.push_back(attrac);
	
	
	//construction d'un champ de vitesse tourbillon
	ArRef<TourbillonVit> tourb = TourbillonVit::NEW();
	tourb->setOrigin(Util3D::Dbl3(0.0,0.0,0.0));
	tourb->setAxis(Util3D::Dbl3(1.0,0.0,0.0));
	
	_tempest->m_vitesses.push_back(tourb);
	
}

void Action::_tempestFollow(double dt)
{
	double x = 0;
	_renderer->globalToLocalPosition(x,x,x);
	_tempest->setLocation(_renderer);
	
	const double angle = M_PI/4; //45deg
	_tempestYaw = ArSystem::realRand()*angle -  0.5*angle;
	_tempestPitch = ArSystem::realRand()*angle -  0.5*angle;
	_tempest->pitch(_tempestPitch);
	_tempest->yaw(_tempestYaw);
	_tempest->translate(_tempestX,0.0,0.0);
}


//----------------------------------------------------------------------------


ArRef<Scheduler>
simulationInit(void)
{
ArRef<Scheduler> sched=RealTimeScheduler::NEW(1e-6);

//viewer
ArRef<Viewer3D> v=Viewer3D::NEW();
v->setCloseAction(Renderer3D::CLOSE_LEAVE);
ArRef<SimpleInteractor> interactor=SimpleInteractor::NEW();
interactor->setTransient(false); // UGLY !
interactor->setRenderer(v);
unsigned int w,h;
Renderer3D::getScreenSize(w,h);
v->setWindowGeometry(50,50,1680,980);
v->setMapped(true);
v->translate(-5.0,0.0,0.0);
v->setBackgroundColor(0.0,0.0,0.0);

ArRef<Action> action = Action::NEW();
action->setRenderer(v);
v->addKeyboardCB(action, &Action::keyboardCB);
//v->setStereoMode(Renderer3D::STEREO_SPLIT);
v->setDecoration(false);
return(sched);
}

int
main(int argc,
     char ** argv)
{
ArSystem arevi(argc,argv);
ArSystem::loadPlugin("OSXImageLoader");
ArSystem::loadPlugin("Imlib2ImageLoader");
ArSystem::loadPlugin("MagickImageLoader");

Action::REGISTER_CLASS();
ParticlesEtForces::REGISTER_CLASS();
Attracteur::REGISTER_CLASS();
TourbillonVit::REGISTER_CLASS();
ArSystem::simulationLoop(&simulationInit);
return(0);
}
