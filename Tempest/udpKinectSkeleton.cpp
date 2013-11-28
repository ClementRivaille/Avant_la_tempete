#include "udpKinectSkeleton.h"

#include "AReVi/Utils/tcpUdp.h"
#include "AReVi/Utils/memoryBlock.h"
#include "AReVi/Utils/memoryStream.h"
#include "ArKinect2/kinectOpenNI2.h"
#include "hLib/hLib.h"

using namespace AReVi;
using namespace hLib;

using std::cout;
using std::cerr;
using std::endl;

AR_CLASS_DEF(UDPKinectSkeleton,ArObject)

UDPKinectSkeleton::UDPKinectSkeleton(ArCW & arCW)
  : ArObject(arCW),
  _skeleton(Skeleton::nullRef()),
  _udp(UDPTransmitter::nullRef()),
  _kinect(KinectOpenNI2::nullRef()),
  _test(0)
{
}

UDPKinectSkeleton::~UDPKinectSkeleton()
{
}

void
UDPKinectSkeleton::initReceiveKinect()
{
//create a skeleton compatible with the kinect
//ArRef<KinectOpenNI2> kinect = _kinect;
//if (!kinect.valid())
//  { 
//  kinect = KinectOpenNI2::NEW(); 
//  cout<<"kinect temporaire pour creer le skeleton. C'est idiot"<<endl;
//  }
//_skeleton = kinect->createPlayerSkeleton();
_skeleton = KinectOpenNI2::createASkeleton();
}

void
UDPKinectSkeleton::initEmitKinect(unsigned int port, int deviceNumber )
{
_kinect = KinectOpenNI2::NEW(deviceNumber);
_skeleton = _kinect->createPlayerSkeleton();
_kinect->setMirror(false);
_kinect->setDebug(false);
_kinect->update();
/*
_kinect->addNewUserCB(thisRef(),&UDPKinectSkeleton::_newUserCB);
_kinect->addLostUserCB(thisRef(),&UDPKinectSkeleton::_lostUserCB);
_kinect->addCalibrationStartedCB(thisRef(),&UDPKinectSkeleton::_calibrationStartedCB);
_kinect->addCalibrationEndedCB(thisRef(),&UDPKinectSkeleton::_calibrationEndedCB);
*/

ArRef<Activity> act = Activity::NEW(1.0/30.0); //30Hz, Kinect 
act->setBehavior(thisRef(), &UDPKinectSkeleton::_action);

_udp = UDPTransmitter::NEW(port);
if (_udp->fail())
  {
  const StlString message = _udp->getErrorMessage();
  cerr<<"Error on upd transmitter creation :"<<message<<endl;
  _udp = UDPTransmitter::nullRef();
  setErrorMessage(message);
  }
}

void
UDPKinectSkeleton::setReceptionPort(unsigned int port)
{
if (_udp.valid())
  {
  ArSystem::removeReadFDPolling(_udp->getSocketFD());
  Scheduler::access()->removePollingCB(thisRef(),&UDPKinectSkeleton::_onPollingUDP);
  }
_udp = UDPTransmitter::NEW(port);
if (_udp->fail())
  {
  const StlString message = _udp->getErrorMessage();
  cerr<<"Error on upd transmitter creation :"<<message<<endl;
  _udp = UDPTransmitter::nullRef();
  setErrorMessage(message);
  }
else
  {
  ArSystem::addReadFDPolling(_udp->getSocketFD());
  Scheduler::access()->addPollingCB(thisRef(),&UDPKinectSkeleton::_onPollingUDP);
  }
}

void
UDPKinectSkeleton::addTarget(
  const StlString & ipString,
  unsigned int port,
  bool OSC,
  unsigned int port2)
{
if (OSC)
  { cout<<"Adding an OSC target"<<endl; }
unsigned int ip;
if (strToIPAddr(ip,ipString))
  {
  Address address;
  address.ip = ip;
  address.port = port;
  address.OSC = OSC; 
  address.port2 = port2;
  _addresses.push_back(address);
  }
}

ArRef<Skeleton>
UDPKinectSkeleton::accessSkeleton()
{
return _skeleton;
}

ArConstRef<Skeleton>
UDPKinectSkeleton::getSkeleton() const
{
return _skeleton;
}

StlMap<StlString, double> &
UDPKinectSkeleton::accessConfidences()
{
return _confidences;
}

bool
UDPKinectSkeleton::_action(ArRef<Activity>, double)
{
if (_kinect.valid())
  {
  _kinect->update();
  _kinect->updatePlayerSkeleton(_skeleton);
  StlMap<StlString,double>::iterator it = _kinect->accessConfidences().begin();
  while (it != _kinect->accessConfidences().end())
    {
    _confidences[it->first]=it->second;
    ++it;
    }
  _transmitCurrentPose();
  _transmitCurrentPoseOSC();
  }
return true;
}

void
UDPKinectSkeleton::_transmitCurrentPose()
{
if (_udp.valid() && _skeleton.valid())
  {
  ArRef<MemoryBlock> memory = MemoryBlock::NEW();
  ArRef<MemoryOStream> stream = MemoryOStream::NEW(memory,0);
  unsigned int nb = _skeleton->getNbJoints();
  stream->writeUInt(nb);
  SerializationDependencies dependencies;
  for (unsigned int i(0);i<nb;++i)
    {
    ArConstRef<Joint> joint = _skeleton->getJoint(i);
    joint->serialize(dependencies,stream); 
    }
  stream->writeInt(_confidences.size());
  StlMap<StlString,double>::iterator it(_confidences.begin());
  while (it != _confidences.end())
    {
    stream->writeString(it->first);
    stream->writeDouble(it->second);
    ++it;
    }

  for (unsigned int i(0);i<_addresses.size();++i)
    {
    if (!_addresses[i].OSC)
      {
      _udp->sendBytes(memory,0,memory->getSize(),
                                 _addresses[i].ip,_addresses[i].port);
      }
    }
  }
}

void
UDPKinectSkeleton::_transmitCurrentPoseOSC()
{
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

  for (unsigned int i(0);i<_addresses.size();++i)
    {
    if (_addresses[i].OSC)
      {
      _udp->sendBytes(memory,0,memory->getSize(),
                                 _addresses[i].ip,_addresses[i].port);
      if (_addresses[i].port2 != 0)
        {
        _udp->sendBytes(memory2,0,memory2->getSize(),
                                 _addresses[i].ip,_addresses[i].port2);
        }
      }
    }
  }
}

void
UDPKinectSkeleton::_writeStringOSC(const StlString & value,
                                   ArRef<AbstractOStream> stream)
{
unsigned int modulo = 4+4*(value.size()/4)-value.size();
//cout<<"size "<<value.size()<<" Modulo "<<modulo<<endl;
//cout<<"Write string OSC [";
for (unsigned int i(0);i<value.size();++i)
  { 
  stream->writeChar(value[i]);
  //cout<<value[i];
  }
for (unsigned int i(0);i<modulo;++i)
  { 
  //cout<<"0";
  stream->writeChar(0);
  }
//cout<<"]"<<endl;
}


void
UDPKinectSkeleton::_onPollingUDP(const Scheduler::PollingEvent & event)
{
StlString str;
unsigned int ipAddr;
int port;
ArRef<MemoryBlock> memory = MemoryBlock::NEW();
while(_udp->receiveBytes(memory,ipAddr,port))
  {
  if (_skeleton.valid())
    {
    ArRef<MemoryIStream> stream = MemoryIStream::NEW(memory,0);
    unsigned int nbJoints;
    stream->readUInt(nbJoints);
    if (nbJoints != _skeleton->getNbJoints())
      { 
      cerr<<"Reads "<<nbJoints<<" joints but skeleton has "
           <<_skeleton->getNbJoints()<<"joints !!! using the min"<<endl; }
    SerializationDependencies dependencies;
    for (unsigned int i(0); i<std::min(nbJoints,_skeleton->getNbJoints()) ;++i)
      {
      ArRef<Joint> joint = _skeleton->accessJoint(i);
      joint->unserialize(dependencies,stream);
      }
    int confidencesSize;
    stream->readInt(confidencesSize);
    for (unsigned int i(0); i<confidencesSize; ++i)
      {
      StlString name;
      stream->readString(name);
      double value;
      stream->readDouble(value);
      _confidences[name] = value;
      }
    }
  }
}

/*
void
UDPKinectSkeleton::_newUserCB(const KinectOpenNI::NewUserEvent & event)
{
cerr<<"UDPKinectSkeleton : New user, number "<<intToStr(event.user)<<endl;
}

void
UDPKinectSkeleton::_lostUserCB(const KinectOpenNI::LostUserEvent & event)
{
cerr<<"UDPKinectSkeleton : Lost user, number "<<intToStr(event.user)<<endl;
}

void
UDPKinectSkeleton::_calibrationStartedCB(const KinectOpenNI::CalibrationStartedEvent & event)
{
cerr<<"UDPKinectSkeleton : Calibration started, user "<<intToStr(event.user)<<endl;
}

void
UDPKinectSkeleton::_calibrationEndedCB(const KinectOpenNI::CalibrationEndedEvent & event)
{
if (event.success)
  { cerr<<"UDPKinectSkeleton : Calibration success, user "<<intToStr(event.user)<<endl; }
else
  { cerr<<"UDPKinectSkeleton : Calibration failed, user "<<intToStr(event.user)<<endl; }
}
*/


