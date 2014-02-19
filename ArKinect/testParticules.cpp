#include "AReVi/arSystem.h"

//pour le viewer
#include "AReVi/Lib3D/viewer3D.h"
#include "AReVi/Lib3D/scene3D.h"
#include "AReVi/activity.h"

//pour le systeme de particules
#include "AReVi/Lib3D/particleSystem.h"

#include <cmath>
#include <algorithm>

//----------------------------------------------------------------------------
// Librairies pour la pour la réception de commandes en OSC
//----------------------------------------------------------------------------
#include <pthread.h>
#include "ReceiveOSCorder.h"

using namespace AReVi;

//-----------------------------------------
//classe de vecteur a 3 dimensions
//-----------------------------------------
class Vecteur
{
protected:
	Util3D::Dbl3 m_coord;
	
public:
	Vecteur(){};
	
	Vecteur(double x, double y, double z)
	{
		m_coord.x = x;
		m_coord.y = y;
		m_coord.z = z;
	}
	
	~Vecteur(){};

	double & x()
	{
		return m_coord.x;
	}
	
	double & y()
	{
		return m_coord.y;
	}
	
	double & z()
	{
		return m_coord.z;
	}
	
	const double & x() const
	{
		return m_coord.x;
	}
	
	const double & y() const
	{
		return m_coord.y;
	}
	
	const double & z() const
	{
		return m_coord.z;
	}
	
	
	
	Vecteur(const Util3D::Dbl3 & v)
	{
		x() = v.x;
		y() = v.y;
		z() = v.z;
	}
	
	Vecteur(const Util3D::Dbl4 & v)
	{
		x() = v.x;
		y() = v.y;
		z() = v.z;
	}
	
	Vecteur(const Vecteur & v)
	{
		x() = v.x();
		y() = v.y();
		z() = v.z();
	}
	
	Vecteur & operator= (const Vecteur & v)
	{
		x() = v.x();
		y() = v.y();
		z() = v.z();
		return (*this);
	}

	Vecteur operator+ (const Vecteur & v) const
	{
		Vecteur res;
		res.x() = x() + v.x();
		res.y() = y() + v.y();
		res.z() = z() + v.z();
		return res;
	}
	
	//operateur soustraction
	Vecteur operator- (const Vecteur & v) const
	{
		Vecteur res;
		res.x() = x() - v.x();
		res.y() = y() - v.y();
		res.z() = z() - v.z();
		return res;
	}
	
	//operateur unaire negatif
	Vecteur operator- () const
	{
		Vecteur res;
		res.x() = -x();
		res.y() = -y();
		res.z() = -z();
		return res;
	}
	
	//produit scalaire
	double operator* (const Vecteur & v) const
	{
		return x()*v.x() + y()*v.y() + z()*v.z();
	}
	
	//multiplication par un scalaire
	Vecteur operator* (double d) const
	{
		Vecteur res;
		res.x() = x()*d;
		res.y() = y()*d;
		res.z() = z()*d;
		return res;
	}
	
	//division par un scalaire
	Vecteur operator/ (double d) const
	{
		Vecteur res;
		res.x() = x()/d;
		res.y() = y()/d;
		res.z() = z()/d;
		return res;
	}
	
	Vecteur & operator+= (const Vecteur & v)
	{
		x() += v.x();
		y() += v.y();
		z() += v.z();
		return (*this);
	}
	
	//produit vectoriel
	Vecteur operator^(const Vecteur & v) const
	{
		Vecteur res;
		res.x() = (*this).y()*v.z() - (*this).z()*v.y();
		res.y() = (*this).z()*v.x() - (*this).x()*v.z();
		res.z() = (*this).x()*v.y() - (*this).y()*v.x();
		return res;
	}
	
	double norm2() const
	{
		return (*this)*(*this);
	}
	
	double norm() const
	{
		return pow(this->norm2(),0.5);
	}
	
	Vecteur normalized() const
	{
		return (*this)/norm();
	}
	
	Util3D::Dbl3 structure() const
	{
		return m_coord;
	}
	
};

//-----------------------------------------
//classe abstraite de potentiels
//-----------------------------------------
class Potentiel : public ArObject
{
public:
	AR_CLASS(Potentiel)
	
	virtual void setOrigin(const Util3D::Dbl3 & pos);
	
	//virtual Vecteur compute(ParticleSystem::Particle & particle) const = 0;
	virtual Vecteur compute(const Util3D::Dbl4 & pos) const = 0;
	
protected:

	Potentiel(ArCW & arCW);

	Util3D::Dbl3 m_origin;
};

AR_CLASS_NOVOID_DEF(Potentiel, ArObject)

Potentiel::Potentiel(ArCW & arCW)
	: ArObject(arCW)
{}

Potentiel::~Potentiel()
{}

void Potentiel::setOrigin(const Util3D::Dbl3 & pos)
{
	m_origin = pos;
}


//-----------------------------------------
//classe concrete de potentiel : attracteur
//-----------------------------------------
//attracteur proportionnel a l'inverse du carre de la distance, type gravitation
class Attracteur : public Potentiel
{
public:
	AR_CLASS(Attracteur)
	AR_CONSTRUCTOR(Attracteur)
	
	virtual Vecteur compute(const Util3D::Dbl4 & pos) const;
	
	virtual void setForce(double f);
	
	virtual void increment();
	
	virtual void decrement();
	
protected:
	double m_facteur;	//parametre de la force d'attraction
};

AR_CLASS_DEF(Attracteur, Potentiel)

Attracteur::Attracteur(ArCW & arCW)
	: Potentiel(arCW),
	  m_facteur(1.0)
{}

Attracteur::~Attracteur()
{}

void Attracteur::setForce(double f)
{
	m_facteur = f;
}

void Attracteur::increment()
{
	m_facteur += 1.0;
}

void Attracteur::decrement()
{
	if (m_facteur >= 1.0)
	{
		m_facteur -= 1.0;
	}
}

//calcul de la force exercee sur la particule
Vecteur Attracteur::compute(const Util3D::Dbl4 & pos) const
{
	//calcul de la distance entre particule et origine
	Vecteur p(pos);
	Vecteur o(m_origin);
	Vecteur distance = o-p;	//dirigee de pos vers origin
	
	//calcul de la force exercee
	//cste gravitationnelle ? masse ?
	Vecteur dirForce = distance.normalized();
	double normForce = m_facteur/distance.norm();
	
	return dirForce * normForce;	
}

//-----------------------------------------
//classe abstraite de champ de vitesses
//-----------------------------------------
class ChampVitesse : public ArObject
{
public:
	AR_CLASS(ChampVitesse)
	
	virtual void setOrigin(const Util3D::Dbl3 & pos);
	
	virtual Vecteur compute(const Util3D::Dbl4 & pos) const = 0;
	
protected:
	ChampVitesse(ArCW & arCW);
	Util3D::Dbl3 m_origin;
};

AR_CLASS_NOVOID_DEF(ChampVitesse, ArObject)

ChampVitesse::ChampVitesse(ArCW & arCW)
	: ArObject(arCW)
{}

ChampVitesse::~ChampVitesse()
{}

void ChampVitesse::setOrigin(const Util3D::Dbl3 & pos)
{
	m_origin = pos;
}

//-----------------------------------------
//classe concrete de champ de vitesses : 
// tourbillon
//
//la direction de la vitesse de la particule est 
//orthogonale au vecteur distance a l'origine.
//la norme de la vitesse est inversement proportionnelle
//a la distance a l'origine
//-----------------------------------------
class TourbillonVit : public ChampVitesse
{
public:
	AR_CLASS(TourbillonVit)
	AR_CONSTRUCTOR(TourbillonVit)
	
	virtual Vecteur compute(const Util3D::Dbl4 & pos) const;
	
	virtual void setAxis(const Util3D::Dbl3 & axis);
	
protected:
	Vecteur m_axis; //axe de rotation du tourbillon
};

AR_CLASS_DEF(TourbillonVit,ChampVitesse)

TourbillonVit::TourbillonVit(ArCW & arCW)
	: ChampVitesse(arCW),
	  m_axis(1.0,0.0,0.0)
{}

TourbillonVit::~TourbillonVit()
{}

void TourbillonVit::setAxis(const Util3D::Dbl3 & axis)
{
	m_axis.x() = axis.x;
	m_axis.y() = axis.y;
	m_axis.z() = axis.z;
}

//calcul de la vitesse instantanee de la particule dans le champ
Vecteur TourbillonVit::compute(const Util3D::Dbl4 & pos) const
{
	//calcul de la distance entre particule et origine
	Vecteur p(pos);
	Vecteur o(m_origin);
	Vecteur distance = p-o;
	
	//calcul de la direction de la vitesse
	Vecteur normalizedDistance = distance.normalized();
	Vecteur normalizedAxis = m_axis.normalized();
	Vecteur directionVit = normalizedDistance ^ normalizedAxis;
	
	//calcul de la norme, fonction de la distance
	//vitesse arbitraire a l'origine
	double normeVit = 1.0/distance.norm2();
	
	//renvoyer la vitesse = dirVit * normVit
	return directionVit*normeVit;
}


//-----------------------------------------
//classe de ParticleSystem modifiee
//-----------------------------------------
class ParticlesEtForces : public ParticleSystem
{
public:
	AR_CLASS(ParticlesEtForces)
	AR_CONSTRUCTOR(ParticlesEtForces)

	StlVector<ArRef<Potentiel> > m_potentiels;	//ensemble des potentiels du systeme de particules
	StlVector<ArRef<ChampVitesse> > m_vitesses; //ensemble des champs de vitesse du systeme de particules
	
protected:	
	//redefinition(s) necessaire(s) a l'utilisation de ParticleEuler
	virtual bool _updateParticle(double dt, ParticleSystem::Particle & particleInOut);
	

	

};

AR_CLASS_DEF(ParticlesEtForces,ParticleSystem)

ParticlesEtForces::ParticlesEtForces(ArCW & arCW)
	: ParticleSystem(arCW)
{
	//potentiel test
	ArRef<Attracteur> attrac = Attracteur::NEW();
	attrac->setOrigin(Util3D::Dbl3(0.0,1.0,1.0));
	m_potentiels.push_back(attrac);
	
	//champ de vitesse test
	ArRef<TourbillonVit> tourb = TourbillonVit::NEW();	//par defaut, m_axis(1.0,0.0,0.0)
	tourb->setAxis(Util3D::Dbl3(1.0,0.0,0.0));
	tourb->setOrigin(Util3D::Dbl3(0.0,1.0,1.0));
	m_vitesses.push_back(tourb);
}

ParticlesEtForces::~ParticlesEtForces()
{}

bool ParticlesEtForces::_updateParticle(double dt, ParticleSystem::Particle & particleInOut)
{
	if(particleInOut.getCollider())
	{
		particleInOut.accessPosition().x = particleInOut.getOldPosition().x
										 + particleInOut.getSpeed().x*dt;
		particleInOut.accessPosition().y = particleInOut.getOldPosition().y
										 + particleInOut.getSpeed().y*dt;
		particleInOut.accessPosition().z = particleInOut.getOldPosition().z
										 + particleInOut.getSpeed().z*dt;
	}
	else
	{
		particleInOut.accessOldPosition().x = particleInOut.getPosition().x;
		particleInOut.accessOldPosition().y = particleInOut.getPosition().y;
		particleInOut.accessOldPosition().z = particleInOut.getPosition().z;
		
		double d = 0.5*dt*dt;
		
		//calculer la force resultante
		Vecteur resultante(0.0,0.0,0.0);
		for(unsigned int i = 0 ; i < m_potentiels.size() ; i++)
		{
			resultante += m_potentiels[i]->compute(particleInOut.getPosition());
		}
		
		//calculer la somme des vitesses appliquees a la particule
		Vecteur vitesses(0.0,0.0,0.0);
		for(unsigned int i = 0 ; i < m_vitesses.size() ; i++)
		{
			vitesses += m_vitesses[i]->compute(particleInOut.getPosition());
		}
		
		//msie a jour de la particule
		particleInOut.accessPosition().x += (particleInOut.getSpeed().x + vitesses.x())*dt + resultante.x()*d;
		particleInOut.accessPosition().y += (particleInOut.getSpeed().y + vitesses.y())*dt + resultante.y()*d;
		particleInOut.accessPosition().z += (particleInOut.getSpeed().z + vitesses.z())*dt + resultante.z()*d;
		particleInOut.accessSpeed().x += vitesses.x() + resultante.x()*dt;
		particleInOut.accessSpeed().y += vitesses.y() + resultante.y()*dt;
		particleInOut.accessSpeed().z += vitesses.z() + resultante.z()*dt;
	}
	particleInOut.accessPosition().a += particleInOut.getSpeed().a*dt;
	return true;
}

//-----------------------------------------
//classe de viewer perso
//-----------------------------------------
class MonViewer : public Viewer3D
{
public:
	AR_CLASS(MonViewer)
	AR_CONSTRUCTOR(MonViewer)
	
	/**
	*  Retrieves the singleton instance of Action. If the singleton doesn't exists, creates it.
	* @return ArRef on Action singleton instance
	*/
	static ArRef<MonViewer> getInstance();

	/**
	* Setter and getter of the p_thread in charge of the messages' reception
	*/
	void setTaskCommand(pthread_t);
	pthread_t getTaskCommand();
	
	virtual bool activate(ArRef<Activity> act, double dt);
	
	void updateAttracteur(bool direction);
	
protected:
	ArRef<Scene3D> m_scene;
	ArRef<ParticlesEtForces> m_particles;
	/** P_thread in charge of the reception of commands */
	pthread_t _taskCommand;
	/** Singleton instance */
	static ArRef<MonViewer> _instance;
	
	virtual void initParticles();
	
	virtual void initChamps();
	
	virtual void updateParticles(double dt);
	
	
};

AR_CLASS_DEF(MonViewer,Viewer3D)
// The instance has to be declared separately to properly work
ArRef<MonViewer> MonViewer::_instance;

MonViewer::MonViewer(ArCW & arCW)
	: Viewer3D(arCW),
	  m_scene(Scene3D::NEW()),
	  m_particles(ParticlesEtForces::NEW())
{	
	//ajout du systeme de particules a la scene
	m_scene->addParticleSystem(m_particles);
	
	//position du systeme de particules dans la scene
	m_particles->setPosition(0,0,0);
	
	//initialisation du systeme de particules
	initParticles();
	
	//ajout de champs
	//initChamps();

	//position de la camera/viewer dans la scene
	setPosition(-10.0,0,0);
	
	//scene a afficher
	selectScene(m_scene);
	
	//creation de l'activite
	ArRef<Activity> activity = Activity::NEW(1.0/30.0);
	activity->setBehavior(thisRef(),&MonViewer::activate);
}

MonViewer::~MonViewer()
{}

/**
 *  Retrieves the singleton instance of MonViewer. If the singleton doesn't exists, creates it.
 * @return ArRef on MonViewer singleton instance
 */
ArRef<MonViewer> MonViewer::getInstance()
{
    if (_instance.null())
    {
        _instance = MonViewer::NEW();
    }
    return _instance;
}

void MonViewer::initParticles()
{
	m_particles->setSection(25.0,25.0,0.0,0.0,true);
	m_particles->setEmissionSpeedDispersion(0.5);
	m_particles->setInitialAngleDispersion(0.1);
	m_particles->setParticleDuration(10.0);
	m_particles->setSize(0.04,0.04);
	m_particles->setSizeDispersion(0.01,0.01);
	m_particles->setGravity(0.0,0.0,0.0);
}

void MonViewer::initChamps()
{
	ArRef<Attracteur> autreAttrac = Attracteur::NEW();
	autreAttrac->setOrigin(Util3D::Dbl3(0.0,-3.0,0.0));
	autreAttrac->setForce(6.0);
	m_particles->m_potentiels.push_back(autreAttrac);
}

void MonViewer::updateParticles(double dt)
{
	//const double angle = 3.14/4.0;
	//double yaw = ArSystem::realRand()*angle -  0.5*angle;
	//double pitch = ArSystem::realRand()*angle -  0.5*angle;
	
	//m_particles->pitch(pitch);
	//m_particles->yaw(yaw);
	
	m_particles->update(dt);
}
bool MonViewer::activate(ArRef<Activity>, double dt)
{
	updateParticles(dt);
	return true;
}

void MonViewer::setTaskCommand(pthread_t thr)
{
	_taskCommand = thr;
}

pthread_t MonViewer::getTaskCommand()
{
	return _taskCommand;
}

void MonViewer::updateAttracteur(bool direction)
{
	ArRef<Attracteur> autreAttrac = ar_down_cast<Attracteur> (m_particles->m_potentiels.back());
	if(direction)
		autreAttrac->increment();
	else
		autreAttrac->decrement();
}

//fin MonViewer

//----------------------------------------------------------------------------
// Fonctions de réception de commandes en OSC
//----------------------------------------------------------------------------

/**
 * Call the function MonViewer::updateAttracteur that increase or reduce the pixels' size.
 * @param a : if > 0, the size is increased, else it's reduced
 */
void updateAttracteur(unsigned int a)
{
	ArRef<MonViewer> viewer = MonViewer::getInstance();
	viewer->updateAttracteur((a > 0));
}

/**
 * Reception of commands from other devices with OSC requests.
 * Receives OSC messages, then calls the associated function with the received value.
 */
void *commandReceiver(void*)
{	
    // We allows the thread to be stopped at the end of the application
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
    
    ReceiveOSCorder receiver(7400);
    
	receiver.addOscFunction("attracteur", updateAttracteur);

    // Starting the reception
	while(1)
	{
       receiver.startCommunication();
	}
	
	return NULL;
}

//----------------------------------------------------------------------------

//-----------------------------------------
//Corps de l'application
//-----------------------------------------

ArRef<MonViewer> viewer;	//globale d'apres conseil du tutoriel / en conflit avec autres programmes

ArRef<Scheduler> simulationInit(void)
{
	ArRef<Scheduler> scd = RealTimeScheduler::NEW(1e-3);
	
	viewer = MonViewer::getInstance();
	viewer->setCloseAction(Window3D::CLOSE_LEAVE);
	
	unsigned int w,h;
	Renderer3D::getScreenSize(w,h);
	viewer->setWindowGeometry(50,50,900,768);
	viewer->setMapped(true);
	viewer->setBackgroundColor(0.0,0.0,0.0);
	
	pthread_t thr;
	//création du thread de réception de commandes OSC
	if(pthread_create(&thr, NULL, commandReceiver, NULL)) {
		fprintf(stderr, "Error creating command receiver thread\n");
	}
	viewer->setTaskCommand(thr);
	
	return scd;
}

int main(int argc, char** argv)
{
	ArSystem arevi(argc,argv);
	MonViewer::REGISTER_CLASS();
	ParticlesEtForces::REGISTER_CLASS();
	Attracteur::REGISTER_CLASS();
	TourbillonVit::REGISTER_CLASS();
	
	ArSystem::simulationLoop(&simulationInit);
	
	//quit the application -> cancel the command receiver thread
	ArRef<MonViewer> viewer = MonViewer::getInstance();
	pthread_cancel(viewer->getTaskCommand());
	pthread_join(viewer->getTaskCommand(), NULL);
	
	return 0;
}
