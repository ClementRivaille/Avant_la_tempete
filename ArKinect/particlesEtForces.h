#include "AReVi/arSystem.h"
#include "AReVi/Lib3D/particleSystem.h"

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

