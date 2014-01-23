#include "AReVi/arSystem.h"

//pour le viewer
#include "AReVi/Lib3D/viewer3D.h"
#include "AReVi/Lib3D/scene3D.h"

//pour le systeme de particules
#include "AReVi/Lib3D/particleSystem.h"

using namespace AReVi;


//-----------------------------------------
//classe de viewer perso
//-----------------------------------------
class MonViewer : public Viewer3D
{
public:
	AR_CLASS(MonViewer)
	AR_CONSTRUCTOR(MonViewer)
	
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
	m_scene->addObject(m_particles);

	//position du systeme de particules dans la scene
	m_particles->setPosition(0,0,0);

	//position de la camera/viewer dans la scene
	setPosition(-10,0,0);
	
	//scene a afficher
	selectScene(m_scene);
}

MonViewer::~MonViewer()
{}



//fin MonViewer
//-----------------------------------------
//Corps de l'application
//-----------------------------------------

ArRef<Viewer3D> viewer;	//globale d'apres conseil du tutoriel / en conflit avec autres programmes

ArRef<Scheduler> simulationInit(void)
{
	ArRef<Sheduler> scd = RealTimeScheduler::NEW(1e-3);
	
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