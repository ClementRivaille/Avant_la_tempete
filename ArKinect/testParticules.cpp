#include "AReVi/arSystem.h"

//pour le viewer
#include "AReVi/Lib3D/viewer3D.h"
#include "AReVi/Lib3D/scene3D.h"
#include "AReVi/activity.h"

//pour le systeme de particules
#include "AReVi/Lib3D/particleSystem.h"

#include <cmath>

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
	
	~Vecteur(){};

	Vecteur(const Util3D::Dbl3 & v)
	{
		m_coord.x = v.x;
		m_coord.y = v.y;
		m_coord.z = v.z;
	}
	
	Vecteur(const Vecteur & v)
	{
		m_coord.x = v.m_coord.x;
		m_coord.y = v.m_coord.y;
		m_coord.z = v.m_coord.z;
	}

	Vecteur operator+ (const Vecteur & v) const
	{
		Vecteur res;
		res.m_coord.x = m_coord.x + v.m_coord.x;
		res.m_coord.y = m_coord.y + v.m_coord.y;
		res.m_coord.z = m_coord.z + v.m_coord.z;
		return res;
	}
	
	Vecteur operator- (const Vecteur & v) const
	{
		Vecteur res;
		res.m_coord.x = m_coord.x - v.m_coord.x;
		res.m_coord.y = m_coord.y - v.m_coord.y;
		res.m_coord.z = m_coord.z - v.m_coord.z;
		return res;
	}
	
	Vecteur operator- () const
	{
		Vecteur res;
		res.m_coord.x = -m_coord.x;
		res.m_coord.y = -m_coord.y;
		res.m_coord.z = -m_coord.z;
		return res;
	}
	
	double operator* (const Vecteur & v) const
	{
		return m_coord.x*v.m_coord.x + m_coord.y*v.m_coord.y + m_coord.z*v.m_coord.z;
	}
	
	Vecteur operator* (double & d) const
	{
		Vecteur res;
		res.m_coord.x = m_coord.x*d;
		res.m_coord.y = m_coord.y*d;
		res.m_coord.z = m_coord.z*d;
		return res;
	}
	
	Vecteur operator/ (double & d) const
	{
		Vecteur res;
		res.m_coord.x = m_coord.x/d;
		res.m_coord.y = m_coord.y/d;
		res.m_coord.z = m_coord.z/d;
		return res;
	}
	
	double norm2() const
	{
		return (*this)*(*this);
	}
	
	double norm() const
	{
		return pow(this->norm2,0.5);
	}
	
	Vecteur normalized() const
	{
		return (*this)/norm();
	}
	
};

//-----------------------------------------
//classe abstraite de potentiels
//-----------------------------------------
class Potentiel
{
public:
	Potentiel();
	virtual ~Potentiel();
	
	virtual void setOrigin(const Util3D::Dbl3 & pos);
	
	virtual const Util3D::Dbl3 & getOrigin() const;
	
	virtual void compute(StlList<ParticlesEtForces::ParticleEuler *> & particles) const = 0;
	
protected:
	Util3D::Dbl3 m_origin;
};

Potentiel::Potentiel()
{}

Potentiel::~Potentiel()
{}

void Potentiel::setOrigin(const Util3D::Dbl3 & pos)
{
	m_origin = pos;
}

const Util3D::Dbl3 & Potentiel::getOrigin() const
{
	return m_origin;
}

//-----------------------------------------
//classe concrete de potentiel : attracteur
//-----------------------------------------
//attracteur proportionnel a l'inverse du carre de la distance, type gravitation
class Attracteur : public Potentiel
{
public:
	virtual void compute(StlList<ParticlesEtForces::ParticleEuler *> & particles) const;
};

void compute(StlList<ParticlesEtForces::ParticleEuler *> & particles) const
{
	//calcul de la force exercee sur chaque particule
	StlList<ParticlesEtForces::ParticleEuler *>::iterator it;
	it=particles.begin();
	while(it != particles.end())
	{
		//calcul de la distance entre particule et origine
		Util3D::Dbl3 distance;
		distance.x = *it->getPosition().x - m_origin.x;
		distance.y = *it->getPosition().y - m_origin.y;
		distance.z = *it->getPosition().z - m_origin.z;
		
		//calcul de la norme au carre de la distance
		double sqNorm = distance.x*distance.x + distance.y*distance.y + distance.z*distance.z;
		
		//calcul de la force exercee
		//cste gravitationnelle ? masse ?   pour l'exemple, tout a 1
		
		
		*it->accessForce += 
	}
}

//-----------------------------------------
//classe de ParticleSystem modifiee
//-----------------------------------------
class ParticlesEtForces : public ParticleSystem
{
public:
	AR_CLASS(ParticlesEtForces)
	AR_CONSTRUCTOR(ParticlesEtForces)
	
	
	//classe de particules capables de retenir la force exerc�e, point de vue eul�rien
	class ParticleEuler : public ParticleSystem::Particle
	{
	public:
	
		ParticleEuler(const ParticleSystem::Particle & p);
	
		inline const Util3D::Dbl3 & getForce(void) const
		{
			return forceResultante;
		}
		inline Util3D::Dbl3 & accessForce(void)
		{
			return forceResultante;
		}
		
	private:
		friend class ParticlesEtForces;
		ParticleEuler(void);
		ParticleEuler(const ParticleEuler & p);
		ParticleEuler & operator=(const Particle & p);
		ParticleEuler & operator=(const ParticleEuler & p);
		
	protected:
		Util3D::Dbl3 forceResultante;
	};
	
	//redefinition(s) necessaire(s) a l'utilisation de ParticleEuler
	virtual bool _updateParticle(double dt, ParticleSystem::Particle & particleInOut);
	
protected:
	StlVector<Potentiel *> m_potentiels;

	//calcul des forces exercees sur les particules baignees dans un champ de potentiel
	void calculForces(StlList<ParticleEuler *> & _partEuler);
};


//-----------------------------------------
//classe de viewer perso
//-----------------------------------------
class MonViewer : public Viewer3D
{
public:
	AR_CLASS(MonViewer)
	AR_CONSTRUCTOR(MonViewer)
	
	virtual bool activate(ArRef<Activity> act, double dt);
	
protected:
	ArRef<Scene3D> m_scene;
	ArRef<ParticleSystem> m_particles;
};

AR_CLASS_DEF(MonViewer,Viewer3D)

MonViewer::MonViewer(ArCW & arCW)
	: Viewer3D(arCW),
	  m_scene(Scene3D::NEW()),
	  m_particles(ParticleSystem::NEW())
{	
	//ajout du systeme de particules a la scene
	m_scene->addParticleSystem(m_particles);

	//position du systeme de particules dans la scene
	m_particles->setPosition(0,0,0);

	//position de la camera/viewer dans la scene
	setPosition(-10,0,0);
	
	//scene a afficher
	selectScene(m_scene);
	
	//creation de l'activite
	ArRef<Activity> activity = Activity::NEW(1.0/30.0);
	activity->setBehavior(thisRef(),&MonViewer::activate);
}

MonViewer::~MonViewer()
{}

bool MonViewer::activate(ArRef<Activity>, double dt)
{
	m_particles->update(dt);
}



//fin MonViewer
//-----------------------------------------
//Corps de l'application
//-----------------------------------------

ArRef<Viewer3D> viewer;	//globale d'apres conseil du tutoriel / en conflit avec autres programmes

ArRef<Scheduler> simulationInit(void)
{
	ArRef<Scheduler> scd = RealTimeScheduler::NEW(1e-3);
	
	viewer = MonViewer::NEW();
	viewer->setCloseAction(Window3D::CLOSE_LEAVE);
	
	unsigned int w,h;
	Renderer3D::getScreenSize(w,h);
	viewer->setWindowGeometry(w-400,0,400,400);
	viewer->setMapped(true);
	
	return scd;
}

int main(int argc, char** argv)
{
	ArSystem arevi(argc,argv);
	MonViewer::REGISTER_CLASS();
	
	ArSystem::simulationLoop(&simulationInit);
	return 0;
}
