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

#include <osg/LightModel>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <DebugHandler.h>
#include <Game.h>
#include <ToyViewer.h>

int main(int argc, char* argv[])
{
    srand(time(0));

    // cache everything
    auto options = new osgDB::Options();
    options->setObjectCacheHint(osgDB::Options::CACHE_ALL);
    osgDB::Registry::instance()->setOptions(options);

    toy::ToyViewer viewer;
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
    // p    capture screen
    // 7    toggle lighting
    // 8    toggle backface culling
    // 9    toggle texture
    // 0    Toggle polygon mode

    viewer.addEventHandler(new toy::DebugHandler(viewer.getCamera()));

    auto statsHandler = new osgViewer::StatsHandler;
    statsHandler->setKeyEventTogglesOnScreenStats(osgGA::GUIEventAdapter::KEY_F1);

    viewer.addEventHandler(statsHandler);
    viewer.addEventHandler(new toy::ViewerDebugHandler(&viewer));

    auto scHander = new osgViewer::ScreenCaptureHandler;
    scHander->setKeyEventTakeScreenShot(osgGA::GUIEventAdapter::KEY_P);
    viewer.addEventHandler(scHander);

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
