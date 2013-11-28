#ifndef UDP_KINECT_SKELETON_H
#define UDP_KINECT_SKELETON_H 1

#include "AReVi/arSystem.h"
#include "AReVi/arObject.h"


namespace hLib {
class Skeleton;
} 

namespace AReVi {
class UDPTransmitter;
class AbstractOStream;
class KinectOpenNI2;

class UDPKinectSkeleton : public ArObject
{
public:
  AR_CLASS(UDPKinectSkeleton)
  AR_CONSTRUCTOR(UDPKinectSkeleton)

public:
  //init mode, deviceNumber starts from 1, cf ArKinect
  //default value 0 is auto detect, aka uses the first detected
  virtual
  void
  initReceiveKinect();

  virtual
  void
  initEmitKinect(unsigned int port, int deviceNumber = 0); //port to construct udp transmitter

  //network
  virtual
  void
  setReceptionPort(unsigned int port); //in reception mode
 
  virtual
  void
  addTarget(
    const StlString & ip,
    unsigned int port,
    bool OSC = false, //will transmit an OSC message
    unsigned int port2 = 0
  ); //in emission mode

  //data
  virtual
  ArRef<hLib::Skeleton> 
  accessSkeleton();

  virtual
  ArConstRef<hLib::Skeleton>
  getSkeleton() const;

  virtual
  StlMap<StlString,double> &
  accessConfidences();

protected:
  virtual
  bool
  _action(ArRef<Activity> activity, double dt);

  virtual
  void
  _transmitCurrentPose();

  virtual
  void
  _onPollingUDP(const Scheduler::PollingEvent & event);

//OSC
  virtual
  void
  _transmitCurrentPoseOSC();
 
  virtual
  void
  _writeStringOSC(const StlString & value, ArRef<AbstractOStream> stream);

//kinect callbacks
/*
  virtual 
  void 
  _newUserCB(const KinectOpenNI::NewUserEvent & evt);

  virtual
  void
  _lostUserCB(const KinectOpenNI::LostUserEvent & evt);

  virtual
  void
  _calibrationStartedCB(const KinectOpenNI::CalibrationStartedEvent & evt);

  virtual
  void 
  _calibrationEndedCB(const KinectOpenNI::CalibrationEndedEvent & evt);
*/

protected:
  ArRef<hLib::Skeleton> _skeleton;
  StlMap<StlString,double> _confidences;
  ArRef<UDPTransmitter> _udp;
  ArRef<KinectOpenNI2> _kinect;

  struct Address {
   unsigned int ip;
   unsigned int port;
   bool OSC;
   unsigned int port2;
  };
  StlVector<Address> _addresses;

  int _test;
};

} //namespace AReVi
#endif // UDP_KINECT_SKELETON_H 

