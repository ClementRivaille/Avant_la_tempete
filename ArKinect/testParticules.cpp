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
	
	Vecteur operator- (const Vecteur & v) const
	{
		Vecteur res;
		res.x() = x() - v.x();
		res.y() = y() - v.y();
		res.z() = z() - v.z();
		return res;
	}
	
	Vecteur operator- () const
	{
		Vecteur res;
		res.x() = -x();
		res.y() = -y();
		res.z() = -z();
		return res;
	}
	
	double operator* (const Vecteur & v) const
	{
		return x()*v.x() + y()*v.y() + z()*v.z();
	}
	
	Vecteur operator* (double d) const
	{
		Vecteur res;
		res.x() = x()*d;
		res.y() = y()*d;
		res.z() = z()*d;
		return res;
	}
	
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
};

AR_CLASS_DEF(Attracteur, Potentiel)

Attracteur::Attracteur(ArCW & arCW)
	: Potentiel(arCW)
{}

Attracteur::~Attracteur()
{}

//calcul de la force exercee sur la particule
Vecteur Attracteur::compute(const Util3D::Dbl4 & pos) const
{
	//calcul de la distance entre particule et origine
	Vecteur p(pos);
	Vecteur o(m_origin);
	Vecteur distance = p-o;
	
	//calcul de la force exercee
	//cste gravitationnelle ? masse ?   pour l'exemple, tout a 1
	Vecteur force =  -distance.normalized() * 1.0/distance.norm2() ;
	
	return force;	
}

//-----------------------------------------
//classe abstraite de champ de vitesses
//-----------------------------------------
class ChampVitesse : public ArObject
{
public:
	AR_CLASS(ChampVitesse)
	
	virtual void setOrigin(const Util3D::Dbl3 & pos);
	
	virtual Vecteur compute(Util3D::Dbl4 & pos) const = 0;
	
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
	
	virtual Vecteur compute(Util3D::Dbl4 & pos) const;
	
	virtual void setAxis(Util3D::Dbl3 & axis);
	
protected:
	Vecteur m_axis; //axe de rotation du tourbillon
};

AR_CLASS_DEF(TourbillonVit,ChampVitesse)

TourbillonVit::TourbillonVit(ArCW & arCW)
	: ChampVitesse(arCW),
	  m_axis(0.0,0.0,1.0)
{}

TourbillonVit::~TourbillonVit()
{}

//calcul de la vitesse instantanee de la particule dans le champ
Vecteur TourbillonVit::compute(Util3D::Dbl4 & pos) const
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
	//vitesse arbitraire à l'origine : 100
	double normeVit = 100.0/distance.norm2();
	
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
	
	//redefinition(s) necessaire(s) a l'utilisation de ParticleEuler
	virtual bool _updateParticle(double dt, ParticleSystem::Particle & particleInOut);
	
protected:
	StlVector<ArRef<Potentiel> > m_potentiels;	//ensemble des potentiels du systeme de particules
	
	StlVector<ArRef<ChampVitesse> > m_vitesses; //ensemble des champs de vitesse du systeme de particules

};

AR_CLASS_DEF(ParticlesEtForces,ParticleSystem)

ParticlesEtForces::ParticlesEtForces(ArCW & arCW)
	: ParticleSystem(arCW)
{
	ArRef<Potentiel> attrac = Attracteur::NEW();
	attrac->setOrigin(Util3D::Dbl3(0.0,5.0,5.0));
	m_potentiels.push_back(attrac);
	
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
		
		//msie a jour de la particule
		particleInOut.accessPosition().x += particleInOut.getSpeed().x*dt+resultante.x()*d;
		particleInOut.accessPosition().y += particleInOut.getSpeed().y*dt+resultante.y()*d;
		particleInOut.accessPosition().z += particleInOut.getSpeed().z*dt+resultante.z()*d;
		particleInOut.accessSpeed().x += resultante.x()*dt;
		particleInOut.accessSpeed().y += resultante.y()*dt;
		particleInOut.accessSpeed().z += resultante.z()*dt;
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
	m_particles->setGravity(0.0,0.0,0.0);
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
	return true;
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
