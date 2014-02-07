#include "particlesEtForces.h"

#include <cmath>
#include <algorithm>

//-----------------------------------------
//classe abstraite de potentiels
//-----------------------------------------
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
	tourb->setOrigin(Util3D::Dbl3(0.0,0.5,0.5));
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
