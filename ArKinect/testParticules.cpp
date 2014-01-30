#include "AReVi/arSystem.h"

//pour le viewer
#include "AReVi/Lib3D/viewer3D.h"
#include "AReVi/Lib3D/scene3D.h"
#include "AReVi/activity.h"

//pour le systeme de particules
#include "AReVi/Lib3D/particleSystem.h"

using namespace AReVi;

//-----------------------------------------
//classe abstraite de potentiels
//-----------------------------------------
class Potentiel
{
public:
	Potentiel();
	virtual ~Potentiel();
	
	virtual compute() const = 0;
	
protected:
	Util3D::Dbl3 m_origin;
};

//-----------------------------------------
//classe de ParticleSystem modifiee
//-----------------------------------------
class ParticlesEtForces : public ParticleSystem
{
public:
	AR_CLASS(ParticlesEtForces)
	AR_CONSTRUCTOR(ParticlesEtForces)
	
	
	//classe de particules capables de retenir la force exercée, point de vue eulérien
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
	void calculForces(SltList<ParticleEuler *> & _partEuler);
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
	ArSystem arevi(arg,argv);
	MonViewer::REGISTER_CLASS();
	
	ArSystem::simulationLoop(&simulationInit);
	return 0;
}