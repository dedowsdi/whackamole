// Game
//      click mole to whack
//
// UI
//      score board
//
// Scene
//      sky
//      lawn
//      burrows

#include <osg/Billboard>
#include <osg/ComputeBoundsVisitor>
#include <osg/LightModel>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <ALContext.h>
#include <ALListener.h>
#include <ALSource.h>
#include <DB.h>
#include <DebugHandler.h>
#include <Game.h>
#include <Math.h>
#include <OsgFactory.h>

int main(int argc, char* argv[])
{
    srand(time(0));

    // cache everything
    auto options = new osgDB::Options();
    options->setObjectCacheHint(osgDB::Options::CACHE_ALL);
    osgDB::Registry::instance()->setOptions(options);

    osgViewer::Viewer viewer;
    sgg.init(argc, argv, &viewer);

    auto camera = viewer.getCamera();
    camera->setName("MainCamera");

    auto lm = new osg::LightModel;
    lm->setLocalViewer(true);

    camera->getOrCreateStateSet()->setAttributeAndModes(lm);

    // Add some debug handlers:
    // f1   Stat
    // f2   Print scene
    // f3   Print render stages
    // f4   Save main camera to main.osgt
    viewer.addEventHandler(new toy::DebugHandler(viewer.getCamera()));

    auto statsHandler = new osgViewer::StatsHandler;
    statsHandler->setKeyEventTogglesOnScreenStats(osgGA::GUIEventAdapter::KEY_F1);

    viewer.addEventHandler(statsHandler);

    auto statesetHandler =
        new osgGA::StateSetManipulator(sgg.getRoot()->getOrCreateStateSet());
    statesetHandler->setKeyEventToggleLighting('7');
    statesetHandler->setKeyEventToggleBackfaceCulling('8');
    statesetHandler->setKeyEventToggleTexturing('9');
    statesetHandler->setKeyEventCyclePolygonMode('0');

    viewer.addEventHandler(statesetHandler);

    viewer.run();

    return 0;
}
