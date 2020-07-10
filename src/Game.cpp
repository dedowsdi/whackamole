#include <Game.h>

#include <cassert>

#include <osg/AnimationPath>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
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

class GameEventHandler : public osgGA::GUIEventHandler
{
public:
    GameEventHandler() {}

    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        switch (ea.getEventType())
        {
            case osgGA::GUIEventAdapter::PUSH:
                switch (ea.getButton())
                {
                    case osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON:
                    {
                        auto mole = getCursorMole(ea);
                        if (mole)
                        {
                            sgg.kickMole(mole);
                        }
                    }
                    break;
                    default:
                        break;
                }
                break;

            case osgGA::GUIEventAdapter::KEYDOWN:
                sgg.restart();
                break;

            case osgGA::GUIEventAdapter::MOVE:
                sgg.highLightMole(getCursorMole(ea));
                break;

            default:
                break;
        }
        return false;  // return true will stop event
    }

private:
    Mole* getCursorMole(const osgGA::GUIEventAdapter& ea)
    {
        osgUtil::LineSegmentIntersector::Intersections iss;
        if (!sgg.getViewer()->computeIntersections(ea, iss))
            return 0;

        auto& np = iss.begin()->nodePath;
        auto iter = std::find_if(np.begin(), np.end(), [](osg::Node* node) -> bool {
            return node->getName().find_first_of("Mole") == 0 && dynamic_cast<Mole*>(node);
        });

        return iter == np.end() ? 0 : static_cast<Mole*>(*iter);
    }
};

osg::ref_ptr<osg::Node> Mole::_drawable;
osg::BoundingBox Mole::_boundingbox;

Mole::Mole(Burrow* burrow) : _burrow(burrow)
{
    addChild(getDrawable());
}

const osg::BoundingBox& Mole::getDrawableBoundingBox()
{
    if (!_boundingbox.valid())
    {
        auto node = getDrawable();
        osg::ComputeBoundsVisitor visitor;
        node->accept(visitor);
        _boundingbox = visitor.getBoundingBox();
    }
    return _boundingbox;
}

osg::Node* Mole::getDrawable()
{
    if (!_drawable)
    {
        _drawable = osgDB::readNodeFile("model/mole.osgt");
    }
    return _drawable;
}

bool Game::run(osg::Object* object, osg::Object* data)
{
    auto visitor = data->asNodeVisitor();
    auto t0 = visitor->getFrameStamp()->getReferenceTime();
    auto deltaTime = _lastTime == 0 ? 0 : t0 - _lastTime;
    _lastTime = t0;

    if (_status == gs_running)
    {
        auto newMole = toy::unitRand() > 0.99;
        if (newMole)
        {
            popMole();
        }

        _timer -= deltaTime;
        _timerText->setText(std::to_string(_timer));

        if (_timer <= 0)
        {
            timeout();
        }
    }

    return traverse(object, data);
}

void Game::createScene()
{
    _hudCamera->addChild(createUI());
    _root->addUpdateCallback(this);
    _root->addEventCallback(new GameEventHandler);
}

void Game::popMole()
{
    static auto moleIndex = 0;

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
    startPos.z() -= Mole::getDrawableBoundingBox().zMax();
    auto endPos = burrow.pos;
    endPos.z() -= Mole::getDrawableBoundingBox().zMin();

    auto mole = new Mole(&burrow);
    mole->setName("Mole" + std::to_string(moleIndex++));
    mole->setMatrix(osg::Matrix::translate(startPos));
    _root->addUpdateCallback(osgf::createTimerUpdateCallback(
        8, [=](osg::Object* object, osg::Object* data) -> void { sgg.removeMole(mole); }));

    auto animPath = new osg::AnimationPath;
    animPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
    osgf::addControlPoints(
        *animPath, 2, 0, 1, mole->getMatrix(), osg::Matrix::translate(endPos));
    osgf::addControlPoints(
        *animPath, 2, 1, 2, osg::Matrix::translate(endPos), mole->getMatrix(), false);

    auto apc = new osg::AnimationPathCallback;
    apc->setAnimationPath(animPath);
    mole->setUpdateCallback(apc);

    _sceneRoot->addChild(mole);
}

void Game::kickMole(Mole* mole)
{
    assert(mole->getNumParents() == 1);

    mole->setKicked(true);
    mole->getBurrow()->active = false;

    // kick it away
    auto translation =
        sphericalRand(200, osg::Vec2(0, osg::PI_2 * 0.8), osg::Vec2(0, 2 * osg::PI));
    auto targetMatrix = mole->getMatrix();
    targetMatrix.preMultTranslate(translation);
    auto duration = 2;

    auto animPath = new osg::AnimationPath;
    animPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
    osgf::addControlPoints(*animPath, 1, 0, duration, mole->getMatrix(), targetMatrix);

    auto apc = new osg::AnimationPathCallback;
    apc->setAnimationPath(animPath);

    mole->setUpdateCallback(apc);

    updateScore(mole->getMatrix().getTrans(), mole->getScore());

    OSG_NOTICE << "Kick " << mole->getName() << std::endl;
}

void Game::removeMole(Mole* mole)
{
    assert(mole->getNumParents() == 1);
    if (!mole->getKicked())
    {
        mole->getBurrow()->active = false;
    }
    mole->getParent(0)->removeChild(mole);
}

void Game::highLightMole(Mole* mole) {}

void Game::updateScore(const osg::Vec3& pos, int score)
{
    _score += score;
    _scoreText->setText(std::to_string(_score));
}

void Game::restart()
{
    _score = 0;
    _scoreText->setText("0");

    _sceneRoot->removeChild(0, _sceneRoot->getNumChildren());
    _sceneRoot->addChild(createLawn());

    _burrowList.clear();
    createBurrows();

    _msg->setNodeMask(0);
    _timer = 30;
    _timerText->setNodeMask(-1);
    _timerText->setText(std::to_string(_timer));

    _status = gs_running;

    _viewer->getCameraManipulator()->home(0);
}

void Game::timeout()
{
    _msg->setNodeMask(-1);
    _msg->setText("Press any key to restart");

    _timerText->setNodeMask(0);
    _status = gs_timeout;
}

osg::Node* Game::createLawn()
{
    auto lawn = new osg::Group;
    lawn->setName("Lawn");

    auto ground = new osg::ShapeDrawable(
        new osg::Box(osg::Vec3(), sceneRadius * 2, sceneRadius * 2, lawnHeight));
    ground->setName("Ground");
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
        graph->setName("Burrow");
        graph->setColor(osg::Vec4(0.2f, 0.2f, 0.2f, 0.2f));

        auto ss = graph->getOrCreateStateSet();
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    }

    auto frame = new osg::MatrixTransform;
    frame->setMatrix(osg::Matrix::translate(pos));
    frame->addChild(graph);
    return Burrow{false, -1, pos, frame};
}

osgText::Text* createText(
    const std::string& name, const std::string& text, int size, const osg::Vec3& pos)
{
    static auto font = osgText::readFontFile("fonts/arial.ttf");

    auto t = new osgText::Text;
    t->setName(name);
    t->setDataVariance(osg::Object::DYNAMIC);
    t->setFont(font);
    t->setCharacterSize(size);
    t->setAxisAlignment(osgText::TextBase::XY_PLANE);
    t->setPosition(pos);
    t->setText(text);
    return t;
}

osg::Node* Game::createUI()
{
    _scoreText = createText("Score", "0", 50, osg::Vec3(20, 20, 0));
    _score = 0;

    _msg = createText("Msg", "Press any key to start game.", 20, osg::Vec3(128, 20, 0));
    _timerText = createText("Timer", "30", 20, osg::Vec3(156, 20, 0));

    auto root = new osg::Group;
    root->addChild(_scoreText);
    root->addChild(_msg);
    root->addChild(_timerText);

    _timerText->setNodeMask(0);

    auto ss = root->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    return root;
}

}  // namespace toy
