#include <Game.h>

#include <osg/AnimationPath>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <Math.h>
#include <OsgFactory.h>

namespace toy
{

const int burrows = 8;
const float sceneRadius = 100;
const float lawnHeight = 2.0f;
const float burrowRadius = 6.0f;
const float burrowHeight = 0.1f;
const float burrowOffset = 0.1f;

class MoleEventHandler : public osgGA::GUIEventHandler
{
public:
    MoleEventHandler(osg::Node* mole) : _mole(mole) {}

    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        switch (ea.getEventType())
        {
            case osgGA::GUIEventAdapter::KEYDOWN:
                switch (ea.getKey())
                {
                    case osgGA::GUIEventAdapter::KEY_F1:
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
        return false;  // return true will stop event
    }

private:
    osg::Node* _mole;
};

bool Game::run(osg::Object* object, osg::Object* data)
{
    auto newMole = toy::unitRand() > 0.99;
    if (newMole)
    {
        popMole();
    }

    return traverse(object, data);
}

void Game::createScene()
{
    _mole = osgDB::readNodeFile("model/mole.osgt");
    osg::ComputeBoundsVisitor cbv;
    _mole->accept(cbv);
    _moleBB = cbv.getBoundingBox();

    _sceneRoot->addChild(createLawn());
    createBurrows();

    _hudCamera->addChild(createUI());
    _root->addUpdateCallback(this);
}

void Game::popMole()
{
    // choose inactive burrow
    std::vector<int> indices;
    for (auto& burrow: _burrowList)
    {
        if (!burrow.active)
        {
            indices.push_back(burrow.index);
        }
    }

    if (indices.empty())
    {
        return;
    }

    auto index = static_cast<int>(toy::unitRand() * indices.size());
    auto& burrow = _burrowList[index];

    // pop mole, play animation
    auto startPos = burrow.pos;
    startPos.z() -= _moleBB.zMax();
    auto endPos = burrow.pos;
    endPos.z() -= _moleBB.zMin();

    auto mole = new osg::MatrixTransform;
    mole->setMatrix(osg::Matrix::translate(startPos));
    mole->addChild(_mole);
    mole->setEventCallback(new MoleEventHandler(mole));
    mole->addUpdateCallback(osgf::createTimerUpdateCallback(
        2, [=](osg::Object* object, osg::Object* data) -> void {
            sgg.hideMole(static_cast<osg::Node*>(object));
        }));
    mole->setUserValue("Burrow", index);

    auto animPath = new osg::AnimationPath;
    animPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
    osgf::addControlPoints(
        *animPath, 2, 0, 1, mole->getMatrix(), osg::Matrix::translate(endPos));
    osgf::addControlPoints(
        *animPath, 2, 0, 1, osg::Matrix::translate(endPos), mole->getMatrix(), false);

    auto apc = new osg::AnimationPathCallback;
    apc->setAnimationPath(animPath);
    mole->setUpdateCallback(apc);

    sgg.getSceneRoot()->addChild(mole);
}

void Game::kickMole(osg::Node* mole) {}

void Game::hideMole(osg::Node* mole) {}

osg::Node* Game::createLawn()
{
    auto lawn = new osg::Group;

    auto ground = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(), sceneRadius * 2, sceneRadius * 2, lawnHeight));
    lawn->addChild(ground);

    return lawn;
}

void Game::createBurrows()
{
    std::vector<osg::Vec3> posList;
    auto maxCenter = sceneRadius - burrowRadius;
    for (auto i = 0; i < burrows; ++i)
    {
        auto pos = osg::Vec3(toy::diskRand(maxCenter), lawnHeight * 0.5 + burrowOffset);
        auto burrow = createBurrow(pos);
        burrow.index = i;
        _sceneRoot->addChild(burrow.node);
        _burrowList.push_back(burrow);
    }
}

Burrow Game::createBurrow(const osg::Vec3& pos)
{
    static osg::ref_ptr<osg::ShapeDrawable> graph;
    if (!graph)
    {
        graph = new osg::ShapeDrawable(
            new osg::Cylinder(osg::Vec3(), burrowRadius, burrowHeight));
        graph->setColor(osg::Vec4(0.2f, 0.2f, 0.2f, 0.2f));

        auto ss = graph->getOrCreateStateSet();
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    }

    auto frame = new osg::MatrixTransform;
    frame->setMatrix(osg::Matrix::translate(pos));
    frame->addChild(graph);
    return Burrow{false, -1, pos, frame};
}

osg::Node* Game::createUI()
{
    return new osg::Group;
}

}  // namespace toy
