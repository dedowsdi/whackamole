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
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osgDB/ReaderWriter>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Billboard>

#include <ALContext.h>
#include <ALListener.h>
#include <ALSource.h>
#include <DB.h>
#include <DebugHandler.h>
#include <Game.h>
#include <Math.h>
#include <OsgFactory.h>

const int burrows = 8;
const float skyDistance = 100;

void createScene();
osg::Node* createLawn();
osg::Node* createBurrow();
osg::Node* createUI();

class GameUpdater : public osg::Callback
{
public:
    bool run(osg::Object* object, osg::Object* data) override
    {
        return traverse(object, data);
    }

    const std::vector<osg::Node*>& getBurrows() const { return _burrowList; }
    void setBurrows(const std::vector<osg::Node*>& v) { _burrowList = v; }

private:
    std::vector<osg::Node*> _burrowList;
};

void createScene()
{
    auto sceneRoot = sgg.getSceneRoot();
    sceneRoot->addChild(createLawn());

    std::vector<osg::Node*> burrowList;
    for (auto i = 0; i < burrows; ++i)
    {
        auto burrow = createBurrow();
        sceneRoot->addChild(createBurrow());
        burrowList.push_back(burrow);
    }

    auto hudCamera = sgg.getHudCamera();
    hudCamera->addChild(createUI());

    auto root = sgg.getRoot();
    auto updater = new GameUpdater();
    root->addUpdateCallback(updater);
}

osg::Node* createUI()
{
    return new osg::Group;
}

osg::Node* createLawn()
{
    auto lawn = new osg::Billboard;
    return lawn;
}

osg::Node* createBurrow()
{
    return new osg::Group;
}

int main(int argc, char* argv[])
{
    srand(time(0));

    // cache everything
    auto options = new osgDB::Options();
    options->setObjectCacheHint(osgDB::Options::CACHE_ALL);
    osgDB::Registry::instance()->setOptions(options);

    osgViewer::Viewer viewer;
    sgg.init(argc, argv, &viewer);

    createScene();

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
