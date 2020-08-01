#include <Game.h>

#include <cassert>
#include <iomanip>

#include <osg/AlphaFunc>
#include <osg/AnimationPath>
#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/ClipNode>
#include <osg/ClipPlane>
#include <osg/ComputeBoundsVisitor>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/LightModel>
#include <osg/LightSource>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/PointSprite>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osg/Switch>
#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/OrbitManipulator>
#include <osgGA/TrackballManipulator>
#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/Layer>
#include <osgTerrain/Locator>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgText/Text>
#include <osgUtil/PerlinNoise>
#include <osgUtil/Tessellator>
#include <osgViewer/Viewer>

#include <ALBuffer.h>
#include <ALSource.h>
#include <Config.h>
#include <DB.h>
#include <GhostManipulator.h>
#include <Lightning.h>
#include <OsgFactory.h>
#include <OsgQuery.h>
#include <Outline.h>
#include <Resource.h>
#include <ToyMath.h>

namespace toy
{

#define TOY_INFO OSG_INFO

osg::Vec3 Burrow::getTopCenter()
{
    return osg::Vec3(0, 0, sgc.getFloat("burrow.height") * 0.5f) * node->getMatrix();
}

Wind::Wind()
{
    mutate();
}

void Wind::update(double dt)
{
    _time -= dt;
    if(_time < 0)
    {
        mutate();
    }
    else if (_time >= _duration * 0.8)
    {
        // fade in
        _strength = mix(0.0f, _amplitude, (_duration - _time) / (_duration * 0.2f) );
    }
    else if(_time <= _duration * 0.2 )
    {
        // fade out
        _strength = mix(0.0f, _amplitude, _time / (_duration * 0.2f) );
    }
    else
    {
        _strength = _amplitude;
    }
}

void Wind::updateUniform(osg::StateSet& ss, int index)
{
    std::string base = "winds[" + std::to_string(index) + "]";

    if (!ss.getUniform(base + ".amplitude"))
    {
        ss.addUniform(new osg::Uniform((base + ".amplitude").c_str(), _strength));
        ss.addUniform(new osg::Uniform((base + ".frequence").c_str(), frequence()));
        ss.addUniform(new osg::Uniform((base + ".phi").c_str(), phi()));
        ss.addUniform(new osg::Uniform((base + ".exponent").c_str(), _exponent));
        ss.addUniform(new osg::Uniform((base + ".direction").c_str(), _direction));
    }
    else
    {
        ss.getUniform(base + ".amplitude")->set(_strength);
        ss.getUniform(base + ".frequence")->set(frequence());
        ss.getUniform(base + ".phi")->set(phi());
        ss.getUniform(base + ".exponent")->set(_exponent);
        ss.getUniform(base + ".direction")->set(_direction);
    }
}

void Wind::mutate()
{
    _amplitude = clamp(gaussRand(sgc.getVec2("wind.amplitude.gauss")), 0.01f, 999.f);
    _exponent = clamp(gaussRand(sgc.getVec2("wind.exponent.gauss")), 0.0f, 999.f);
    _speed = clamp(gaussRand(sgc.getVec2("wind.speed.gauss")), 0.0f, 999.f);
    _length = clamp(gaussRand(sgc.getVec2("wind.length.gauss")), 0.0f, 99999.f);;
    _duration = clamp(gaussRand(sgc.getVec2("wind.duration")), 0.0f, 99999.f);;
    _time = _duration;
    _direction = circularRand(1.0f);
    _strength = 0;
}

class GameEventHandler : public osgGA::GUIEventHandler
{
public:
    GameEventHandler() {}

    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        switch (ea.getEventType())
        {
            case osgGA::GUIEventAdapter::FRAME:
            {
                auto mole = getCursorMole(ea);
                sgg.highlightMole(mole);
                sgg.moveCursor(ea.getX(), ea.getY());
                sgg.flashCursor(mole != 0);
            }
            break;

            case osgGA::GUIEventAdapter::PUSH:
                switch (ea.getButton())
                {
                    case osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON:
                    {
                        if (sgg.getStatus() == Game::gs_running)
                        {
                            auto mole = getCursorMole(ea);
                            if (mole)
                            {
                                sgg.whackMole(mole);
                            }
                        }
                    }
                    break;
                    default:
                        break;
                }
                break;

            case osgGA::GUIEventAdapter::KEYDOWN:
                switch (ea.getKey())
                {
                    case osgGA::GUIEventAdapter::KEY_R:
                        sgg.restart();
                        break;

                    default:
                        break;
                }
                break;

            case osgGA::GUIEventAdapter::MOVE:
                break;

            case osgGA::GUIEventAdapter::RESIZE:
                sgg.resize(ea.getWindowWidth(), ea.getWindowHeight());
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
        if (!sgg.getViewer()->computeIntersections(ea, iss, nb_raytest))
            return 0;

        // intersections are sorted by ratio, 1st one is the nearest. Don't use
        // LIMIT_NEAREST, it's used for bounding sphere intersection test.
        auto& np = iss.begin()->nodePath;
        auto iter = std::find_if(np.begin(), np.end(), [](osg::Node* node) -> bool {
            return node->getName().find_first_of("Mole") == 0 && dynamic_cast<Mole*>(node);
        });

        return iter == np.end() ? 0 : static_cast<Mole*>(*iter);
    }
};

osg::ref_ptr<osg::Node> Mole::_model;
osg::ref_ptr<osg::Node> Mole::_burnedModel;
osg::BoundingBox Mole::_boundingbox;

Mole::Mole(Burrow* burrow) : _burrow(burrow)
{
    auto outline = new Outline;
    outline->setWidth(6);
    outline->setColor(osg::Vec4(1, 0.5, 0, 1));
    outline->setEnableCullFace(false);
    outline->setPolygonModeFace(osg::PolygonMode::FRONT_AND_BACK);

    outline->addChild(getModel());

    _switch = new osg::Switch;
    _switch->addChild(getModel(), true);
    _switch->addChild(getBurnedModel(), false);
    _switch->addChild(outline, false);

    addChild(_switch);
    setDataVariance(osg::Object::DYNAMIC);
}

void Mole::setKicked(bool v)
{
    _kicked = v;
    _switch->setSingleChildOn(v ? 1 : 0);
}

void Mole::setHighlighted(bool v)
{
    _highlighted = v;
    _switch->setSingleChildOn(v ? 2 : 0);
}

osg::Node* Mole::getModel()
{
    if (!_model)
    {
        auto node = osgDB::readNodeFile("model/mole.osgt");

        // scale to moleSize
        osg::ComputeBoundsVisitor visitor;
        node->accept(visitor);
        auto bb = visitor.getBoundingBox();
        auto xyRadius =
            osg::Vec2(bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin()).length() * 0.5;
        auto scale = sgc.getFloat("mole.size") / xyRadius;

        osg::Matrix m = osg::Matrix::scale(scale, scale, scale);
        m.preMultTranslate(-bb.center());

        auto frame = new osg::MatrixTransform;
        frame->setMatrix(m);
        frame->addChild(node);

        auto ss = frame->getOrCreateStateSet();
        ss->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

        // ss->setAttributeAndModes(
        //     new osg::CullFace, osg::StateAttribute::OFF |
        //     osg::StateAttribute::PROTECTED);

        // ss->setMode(
        //     GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);

        _model = frame;
    }
    return _model;
}

osg::Node* Mole::getBurnedModel()
{
    if (!_burnedModel)
    {
        auto burned = osg::clone(getModel(), osg::CopyOp::DEEP_COPY_NODES);

        // change body material
        {
            auto nodes = osgq::searchNodeByMaterial(*burned, "Mat.1");
            if (nodes.empty())
            {
                OSG_WARN << "Failed to search body node" << std::endl;
            }
            else
            {
                auto mtl = new osg::Material;
                mtl->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.1, 0.1, 1));

                // Be careful here, 0 is original parent leaf
                auto leaf = nodes.front()->getParent(1);
                auto ss = leaf->getOrCreateStateSet();
                ss->setAttributeAndModes(
                    mtl, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            }
        }

        // change hat material
        {
            auto nodes = osgq::searchNode(*burned, "Null.002_Mesh.002");
            if (nodes.empty())
            {
                OSG_WARN << "Failed to search hat node" << std::endl;
            }
            else
            {
                auto mtl = new osg::Material;
                mtl->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.1, 0.1, 1));

                auto ss = nodes.front()->getOrCreateStateSet();
                ss->setAttributeAndModes(
                    mtl, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            }
        }

        _burnedModel = burned;
    }

    return _burnedModel;
}

bool Game::run(osg::Object* object, osg::Object* data)
{
    auto visitor = data->asNodeVisitor();
    auto t0 = visitor->getFrameStamp()->getSimulationTime();
    _deltaTime = _lastTime == 0 ? 0 : t0 - _lastTime;
    _lastTime = t0;

    if (_status == gs_running && _deltaTime > 0)
    {
        auto newMole = toy::unitRand() < _popRate;
        if (newMole)
        {
            popMole();
        }

        _timer -= _deltaTime;
        std::stringstream ss;
        ss << std::setprecision(2) << std::fixed << std::showpoint << _timer;
        _timerText->setText(ss.str());

        if (_timer <= 0)
        {
            timeout();
        }
    }

    return traverse(object, data);
}

void Game::preInit()
{
    osg::DisplaySettings::instance()->setMinimumNumStencilBits(1);

    auto camera = getMainCamera();
    camera->setCullingMode(
        camera->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING);
    camera->setClearMask(camera->getClearMask() | GL_STENCIL_BUFFER_BIT);
    camera->setClearStencil(0);

    _manipulator = new osgGA::KeySwitchMatrixManipulator;
    _manipulator->addMatrixManipulator(
        osgGA::GUIEventAdapter::KEY_F7, "Trackball", new osgGA::TrackballManipulator);
    _manipulator->addMatrixManipulator(
        osgGA::GUIEventAdapter::KEY_F8, "Ghost", new GhostManipulator);

    _viewer->setCameraManipulator(_manipulator);
}

void Game::postInit()
{
    _viewer->home();

    auto wsize = osgq::getWindowRect(*_viewer);
    resize(wsize.z(), wsize.w());
}

void Game::createScene()
{
    // don't add reload and resize related code here
    _hudCamera->addChild(createUI());

    _root->addUpdateCallback(this);
    _root->addEventCallback(new GameEventHandler);

    createStartAnimation();

    setUseCursor(false);
}

void Game::popMole()
{
    static auto moleIndex = 0;

    // choose inactive burrow
    std::vector<int> indices;
    for (auto i = 0; i < _burrowList.size(); ++i)
    {
        auto& burrow = _burrowList[i];
        if (!burrow.active)
        {
            indices.push_back(i);
        }
    }

    if (indices.empty())
    {
        return;
    }

    auto index = static_cast<int>(toy::unitRand() * indices.size());
    auto& burrow = _burrowList[indices[index]];
    burrow.active = true;

    auto mole = new Mole(&burrow);
    mole->setNodeMask(nb_real_object);
    mole->setName("Mole" + std::to_string(moleIndex++));

    osg::ComputeBoundsVisitor visitor;
    mole->accept(visitor);
    auto bb = visitor.getBoundingBox();

    // pop mole, play animation
    auto startPos = burrow.getTopCenter();
    startPos -= burrow.normal * (bb.zMax() + 0.1);
    auto endPos = burrow.getTopCenter();
    endPos -= burrow.normal * (bb.zMin() - 2);

    auto rot = osg::Matrix::rotate(osg::Z_AXIS, burrow.normal);

    // face mole to camera, rotate along local up
    auto eye = getMainCamera()->getInverseViewMatrix().getTrans();
    auto ev = eye - startPos;
    ev = rot * ev;  // get local ev
    auto theta = std::atan2(ev.x(), -ev.y());
    rot = osg::Matrix::rotate(theta, osg::Z_AXIS) * rot;

    mole->setMatrix(rot * osg::Matrix::translate(startPos));

    auto animPath = new osg::AnimationPath;
    animPath->setLoopMode(osg::AnimationPath::NO_LOOPING);

    auto durationRange = sgc.getVec2("mole.durationRange");
    auto duration = linearRand(durationRange.x(), durationRange.y());
    mole->setScore(100 * duration / 1.0f);

    osgf::addControlPoints(*animPath, 2, 0, duration * 0.35f, mole->getMatrix(),
        rot * osg::Matrix::translate(endPos));
    osgf::addControlPoints(*animPath, 2, duration * 0.35f, duration * 0.65f,
        rot * osg::Matrix::translate(endPos), rot * osg::Matrix::translate(endPos));
    osgf::addControlPoints(*animPath, 2, duration * 0.65f, duration,
        rot * osg::Matrix::translate(endPos), mole->getMatrix(), false);

    auto apc = new osg::AnimationPathCallback;
    apc->setAnimationPath(animPath);
    mole->setUpdateCallback(apc);

    _sceneRoot->addChild(mole);
    auto removeCallback = osgf::createTimerUpdateCallback(
        duration, [=](osg::Object* object, osg::Object* data) { sgg.removeMole(mole); });
    _sceneRoot->addUpdateCallback(removeCallback);
    _removeMoleCallbacks.insert(std::make_pair(mole, removeCallback));
    TOY_INFO << "pop mole " << mole->getName() << " " << mole << std::endl;
}

void Game::whackMole(Mole* mole)
{
    assert(mole->getNumParents() == 1);

    mole->setNodeMask(nb_unreal_object);
    mole->setKicked(true);
    mole->getBurrow()->active = false;

    auto removeCallback = _removeMoleCallbacks.find(mole);
    assert(removeCallback != _removeMoleCallbacks.end());
    _sceneRoot->removeUpdateCallback(removeCallback->second);

    if (_cursorMole.get() == mole)
    {
        _cursorMole = osg::ref_ptr<Mole>();
    }

    auto pos = mole->getMatrix().getTrans();

    playWhackAnimation(pos);

    popScore(pos, mole->getScore());

    explode(pos);

    // play sound
    auto sound = new ALSource(osgDB::readALBufferFile("sound/hit.wav"));
    playSound(mole->getBurrow()->node, sound);

    // whack it away
    auto translation =
        sphericalRand(300, osg::Vec2(0, osg::PI_2 * 0.8), osg::Vec2(0, 2 * osg::PI));
    auto targetMatrix = mole->getMatrix();
    targetMatrix.preMultTranslate(translation);
    auto duration = 2;

    auto animPath = new osg::AnimationPath;
    animPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
    osgf::addControlPoints(*animPath, 1, 0, duration, mole->getMatrix(), targetMatrix);

    auto apc = new osg::AnimationPathCallback;
    apc->setAnimationPath(animPath);

    mole->setUpdateCallback(apc);

    _sceneRoot->addUpdateCallback(osgf::createTimerUpdateCallback(
        duration, [=](osg::Object* object, osg::Object* data) { sgg.removeMole(mole); }));

    updateScore(mole->getMatrix().getTrans(), mole->getScore());

    TOY_INFO << "Whack " << mole->getName() << " " << mole << std::endl;
}

void Game::highlightMole(Mole* mole)
{
    if (mole == _cursorMole)
        return;

    if (_cursorMole)
    {
        _cursorMole->setHighlighted(false);
    }

    if (mole)
    {
        mole->setHighlighted(true);
    }
    _cursorMole = mole;
}

void Game::removeMole(Mole* mole)
{
    TOY_INFO << "Remove mole " << mole->getName() << " " << mole << std::endl;
    _removeMoleCallbacks.erase(mole);
    if (mole->getNumParents() != 1)
    {
        OSG_NOTICE << mole->getName() << " has " << mole->getNumParents() << " parents"
                   << std::endl;
    }
    assert(mole->getNumParents() == 1);

    if (_cursorMole.get() == mole)
    {
        _cursorMole = osg::ref_ptr<Mole>();
    }

    if (!mole->getKicked())
    {
        mole->getBurrow()->active = false;
    }
    mole->getParent(0)->removeChild(mole);
}

void Game::updateScore(const osg::Vec3& pos, int score)
{
    _score += score;
    _scoreText->setText(std::to_string(_score));
}

void Game::restart()
{
    OSG_NOTICE << "Restart game" << std::endl;
    sgc.reload();

    // clear resource observer, but leave static ones
    _observer->clear();
    auto lprg = Lightning::getBillboardProgram();
    if (lprg)
    {
        _observer->addResource(*lprg);
    }

    _sceneRoot->removeChild(0, _sceneRoot->getNumChildren());
    _sceneRoot->setUpdateCallback(0);
    _removeMoleCallbacks.clear();
    _root->removeChild(_reflectRttCamera);
    _root->removeChild(_refractRttCamera);

    auto lm = new osg::LightModel;
    lm->setLocalViewer(true);
    lm->setAmbientIntensity(sgc.getVec4("scene.ambient"));
    _sceneRoot->getOrCreateStateSet()->setAttributeAndModes(lm);
    _sceneRoot->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
    _sceneRoot->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

    auto light = _viewer->getLight();
    light->setDiffuse(sgc.getVec4("scene.headlight.diffuse"));
    light->setSpecular(sgc.getVec4("scene.headlight.specular"));
    light->setAmbient(sgc.getVec4("scene.headlight.ambient"));

    _sceneRadius = sgc.getFloat("scene.radius");
    _sceneHeight = sgc.getFloat("scene.height");
    _popRate = sgc.getFloat("mole.popRate");

    _score = 0;
    _scoreText->setText("0");

    auto camera = getMainCamera();
    camera->setClearColor(osg::Vec4(0.1, 0.1, 0.1, 0.1));

    createStarfield();
    createTerrain();
    createPool();
    createMeadow();

    _burrowList.clear();
    createBurrows();

    hide(_msg);
    _timer = 60;
    _totalTime = 60;
    _timerText->setText(std::to_string(_timer));

    _scoreText->setNodeMask(nb_ui);
    _timerText->setNodeMask(nb_ui);
    _timerBar->setNodeMask(nb_ui);

    _status = gs_running;

    auto kcm =
        static_cast<osgGA::KeySwitchMatrixManipulator*>(_viewer->getCameraManipulator());
    kcm->selectMatrixManipulator(1);
    auto manipulator = static_cast<GhostManipulator*>(kcm->getCurrentMatrixManipulator());
    manipulator->setWalkSpeed(sgc.getFloat("camera.walkSpeed"));
    manipulator->setCameraHeight(sgc.getFloat("camera.height"));

    auto xy = sgc.getVec2("camera.startXY");
    auto tp = getTerrainPoint(xy.x(), xy.y());
    tp.z() += sgc.getFloat("camera.height");
    manipulator->setHomePosition(tp, osg::Vec3(), osg::Z_AXIS);
    _viewer->home();

    auto wsize = osgq::getWindowRect(*_viewer);
    resize(wsize.z(), wsize.w());
}

void Game::timeout()
{
    _msg->setNodeMask(nb_ui);
    _msg->setText("Press r to start new game.");
    _status = gs_timeout;
}

void Game::resize(int width, int height)
{
    // resize default render target
    {
        auto ss = _root->getOrCreateStateSet();
        ss->addUniform(new osg::Uniform("render_target_size", osg::Vec2(width, height)));
        ss->addUniform(new osg::Uniform("render_target_scale", 1.0f));
    }

    // reset hudcamera projection
    osg::Matrix m;
    m.makeOrtho(0, width, 0, height, -1, 1);
    _hudCamera->setProjectionMatrix(m);

    // resize ui
    auto barSize = sgc.getVec2("ui.bar.size");
    barSize = osg::componentMultiply(barSize, osg::Vec2(width, height));
    auto y = sgc.getFloat("ui.bar.y");
    auto x = (width - barSize.x()) * 0.5f;

    // resize time bar
    auto vertices = static_cast<osg::Vec3Array*>(_timerBar->getVertexArray());
    auto corner = osg::Vec3(x, y, 0);
    auto widthVec = osg::Vec3(barSize.x(), 0, 0);
    auto heightVec = osg::Vec3(0, barSize.y(), 0);
    (*vertices)[0] = corner + heightVec;
    (*vertices)[1] = corner;
    (*vertices)[2] = corner + widthVec;
    (*vertices)[3] = corner + widthVec + heightVec;
    vertices->dirty();
    _timerBar->dirtyGLObjects();
    _timerBar->dirtyBound();

    auto ss = _timerBar->getOrCreateStateSet();
    ss->addUniform(new osg::Uniform("size", barSize));

    y += barSize.y() + 2;

    // place text
    _timerText->setPosition(osg::Vec3(x + barSize.x(), y, 0));
    _timerText->setAlignment(osgText::Text::RIGHT_BOTTOM);

    _scoreText->setPosition(osg::Vec3(x, y, 0));

    _msg->setPosition(osg::Vec3(x, height - 16, 0));
    _msg->setAlignment(osgText::Text::LEFT_TOP);

    if (_status == gs_init)
    {
        return;
    }

    // resize rtt cameras.
    // I start this with 0.5, but it cause glitches in the pool edge.
    auto scale = 1.0f;

    _reflectRttCamera->resize(width * scale, height * scale);
    {
        auto ss = _reflectRttCamera->getOrCreateStateSet();
        ss->addUniform(new osg::Uniform(
            "render_target_size", osg::Vec2(width * scale, height * scale)));
        ss->addUniform(new osg::Uniform("render_target_scale", scale));
    }

    _refractRttCamera->resize(width * scale, height * scale);
    {
        auto ss = _refractRttCamera->getOrCreateStateSet();
        ss->addUniform(new osg::Uniform(
            "render_target_size", osg::Vec2(width * scale, height * scale)));
        ss->addUniform(new osg::Uniform("render_target_scale", scale));
    }
}

void Game::moveCursor(float x, float y)
{
    _cursorFrame->setMatrix(osg::Matrix::translate(osg::Vec3(x, y, 0)));
}

void Game::flashCursor(bool v)
{
    static bool lastFlash = false;
    if (lastFlash == v)
    {
        return;
    }
    lastFlash = v;

    auto ss = _cursorGeom->getOrCreateStateSet();
    ss->getUniform("flash")->set(lastFlash ? 1 : 0);
}

void Game::hide(osg::Node* node)
{
    node->setNodeMask(0);
}

osg::Vec3 Game::getTerrainPoint(float x, float y)
{
    auto tile = _terrain->getTile(osgTerrain::TileID(0, 0, 0));
    auto layer = static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    auto ndcX = (x + _sceneRadius) * 0.5 / _sceneRadius;
    auto ndcY = (y + _sceneRadius) * 0.5 / _sceneRadius;

    float h;
    if (layer->getInterpolatedValue(ndcX, ndcY, h))
        return osg::Vec3(x, y, h);
    else
        return osg::Vec3();
}

osg::Vec3 Game::getTerrainNormal(float x, float y)
{
    auto tile = _terrain->getTile(osgTerrain::TileID(0, 0, 0));
    auto layer = static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    auto heightField = layer->getHeightField();

    auto ndcX = (x + _sceneRadius) * 0.5 / _sceneRadius;
    auto ndcY = (y + _sceneRadius) * 0.5 / _sceneRadius;
    auto i = std::round(heightField->getNumColumns() * ndcX);
    auto j = std::round(heightField->getNumRows() * ndcY);
    return _heightField->getNormal(i, j);
}

Game::Game() {}

Game::~Game() {}

void Game::createTerrain()
{
    _heightField = new osg::HeightField;

    // make sure no scale happens between heightfiled and heightfield layer, otherwise
    // HeightField::getNormal will break
    auto rows = sgc.getInt("terrain.rows");
    auto cols = sgc.getInt("terrain.cols");
    auto xInterval = _sceneRadius * 2 / cols;
    auto yInterval = _sceneRadius * 2 / rows;

    _heightField->allocate(rows, cols);
    _heightField->setXInterval(xInterval);
    _heightField->setYInterval(yInterval);
    _heightField->setOrigin(osg::Vec3(-_sceneRadius, -_sceneRadius, 0));

    osgUtil::PerlinNoise pn;
    pn.SetNoiseFrequency(64);

    auto yStep = 0.03;
    auto xStep = 0.03;
    auto noiseFactor = osg::Vec2(linearRand(-100.0f, 100.0f), linearRand(-100.0f, 100.0f));

    auto poolRadius = sgc.getFloat("pool.radius");
    auto poolBottomRadius = sgc.getFloat("pool.bottomRadius");
    auto poolDepth = sgc.getFloat("pool.depth");

    for (int y = 0; y < rows; ++y)
    {
        noiseFactor[1] += yStep;
        noiseFactor[0] = 0;

        for (int x = 0; x < cols; ++x)
        {
            noiseFactor[0] += xStep;
            auto h =
                pn.PerlinNoise2D(noiseFactor[0], noiseFactor[1], 2, 2, 3) * _sceneHeight;

            // dig pool
            auto vertex = _heightField->getVertex(x, y);
            auto l = osg::Vec2(vertex.x(), vertex.y()).length();
            if (l < poolRadius)
            {
                h -= mix(
                    0.0f, poolDepth, 1.0f - smoothstep(poolBottomRadius, poolRadius, l));
            }

            _heightField->setHeight(x, y, h);
        }
    }

    auto locator = new osgTerrain::Locator;
    locator->setCoordinateSystemType(osgTerrain::Locator::GEOGRAPHIC);
    locator->setTransformAsExtents(
        -_sceneRadius, -_sceneRadius, _sceneRadius, _sceneRadius);

    auto layer = new osgTerrain::HeightFieldLayer(_heightField);
    layer->setLocator(locator);

    auto clayer = new osgTerrain::ImageLayer(osgDB::readImageFile("texture/ground.jpg"));

    auto tile = new osgTerrain::TerrainTile;
    tile->setTerrainTechnique(new osgTerrain::GeometryTechnique);
    tile->setTileID(osgTerrain::TileID(0, 0, 0));
    tile->setElevationLayer(layer);
    tile->setColorLayer(0, clayer);

    _terrain = new osgTerrain::Terrain;
    _terrain->setNodeMask(nb_terrain);
    tile->setTerrain(_terrain);
    _terrain->addChild(tile);

    auto ss = _terrain->getOrCreateStateSet();

    auto material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.75, 0.75, 0.75, 1));
    ss->setAttributeAndModes(material);

    _sceneRoot->addChild(_terrain);
}

class FishUpdater : public osg::Callback
{
public:
    bool run(osg::Object* object, osg::Object* data) override
    {
        auto fish = static_cast<osg::MatrixTransform*>(object);
        auto dt = sgg.getDeltaTime();
        _time -= dt;
        if (_time <= 0)
        {
            reset(*fish);
        }
        else
        {
            fish->setMatrix(fish->getMatrix() * osg::Matrix::translate(_vel * dt));
        }
        return traverse(object, data);
    }

private:
    void reset(osg::MatrixTransform& fish)
    {
        auto radius = sgc.getFloat("pool.bottomRadius");
        auto gauss = sgc.getVec2("fish.speed.gauss");

        // reset velocity
        auto pos = fish.getMatrix().getTrans();
        auto targetPos = osg::Vec3(diskRand(radius), pos.z());
        auto speed = clamp(gaussRand(gauss.x(), gauss.y()), 0.01f, 999.0f);
        _vel = targetPos - pos;
        auto distance = _vel.normalize();
        _vel *= speed;
        _time = distance / speed;

        // reset rotation
        auto frame = osg::Matrix::rotate(-osg::Y_AXIS, _vel);
        frame.postMultTranslate(pos);
        fish.setMatrix(frame);
    }

    float _time = 0;
    osg::Vec3 _vel;
};

class PoolCreatureCullCallback : public osg::Callback
{
public:
    bool run(osg::Object* object, osg::Object* data) override
    {
        auto visitor = data->asNodeVisitor()->asCullVisitor();
        if (visitor->getCurrentCamera() == sgg.getMainCamera())
        {
            return false;
        }
        return traverse(object, data);
    }
};

void Game::createPool()
{
    auto radius = sgc.getFloat("pool.radius");
    auto top = sgc.getFloat("pool.top");

    // reflect
    {
        _reflectMap = osgf::createTexture2D(GL_RGBA, 1, 1, osg::Texture::LINEAR,
            osg::Texture::LINEAR, osg::Texture::REPEAT, osg::Texture::REPEAT);
        _reflectMap->setResizeNonPowerOfTwoHint(false);

        _reflectRttCamera =
            osgf::createRttCamera(0, 0, 1, 1, osg::Camera::FRAME_BUFFER_OBJECT);
        _reflectRttCamera->setName("ReflectRttCamera");
        _reflectRttCamera->setNodeMask(nb_above_waterline);
        _reflectRttCamera->attach(osg::Camera::COLOR_BUFFER, _reflectMap);
        _reflectRttCamera->attach(
            osg::Camera::PACKED_DEPTH_STENCIL_BUFFER, GL_DEPTH_STENCIL_EXT);
        _reflectRttCamera->setCullMask(nb_above_waterline);
        _reflectRttCamera->setClearMask(
            GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        _reflectRttCamera->setCullingMode(_reflectRttCamera->getCullingMode() &
                                          ~osg::CullSettings::SMALL_FEATURE_CULLING);
        _reflectRttCamera->setUpdateCallback(osgf::getPruneCallback());
        _reflectRttCamera->setEventCallback(osgf::getPruneCallback());
        _root->addChild(_reflectRttCamera);

        auto frame = new osg::MatrixTransform;
        auto m = osg::Matrix::translate(osg::Vec3(0, 0, top));
        m.preMultScale(osg::Vec3(1, 1, -1));
        m.preMultTranslate(osg::Vec3(0, 0, -top));
        frame->setMatrix(m);

        _reflectRttCamera->addChild(frame);

        auto clipNode = new osg::ClipNode;
        auto clipPlane = new osg::ClipPlane;
        clipPlane->setClipPlaneNum(0);
        clipPlane->setClipPlane(0, 0, 1, -top);
        clipNode->addClipPlane(clipPlane);

        frame->addChild(clipNode);

        auto ss = frame->getOrCreateStateSet();
        ss->setMode(
            GL_CLIP_PLANE0, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        ss->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

        frame->addChild(_sceneRoot);
    }

    // refract{
    {
        _refractMap = osgf::createTexture2D(GL_RGBA, 1, 1, osg::Texture::LINEAR,
            osg::Texture::LINEAR, osg::Texture::CLAMP_TO_EDGE, osg::Texture::CLAMP_TO_EDGE);
        _refractMap->setResizeNonPowerOfTwoHint(false);
        _depthMap = osgf::createTexture2D(GL_DEPTH_COMPONENT, 1, 1, osg::Texture2D::LINEAR,
            osg::Texture2D::NEAREST, osg::Texture::CLAMP_TO_EDGE,
            osg::Texture::CLAMP_TO_EDGE);
        _depthMap->setResizeNonPowerOfTwoHint(false);

        _refractRttCamera =
            osgf::createRttCamera(0, 0, 1, 1, osg::Camera::FRAME_BUFFER_OBJECT);
        _refractRttCamera->setNodeMask(nb_below_waterline);
        _refractRttCamera->setName("RefractRttCamera");
        _refractRttCamera->attach(osg::Camera::COLOR_BUFFER, _refractMap);
        _refractRttCamera->attach(osg::Camera::DEPTH_BUFFER, _depthMap);
        _refractRttCamera->setCullMask(nb_below_waterline);
        _refractRttCamera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        _refractRttCamera->setUpdateCallback(osgf::getPruneCallback());
        _refractRttCamera->setEventCallback(osgf::getPruneCallback());
        _root->addChild(_refractRttCamera);

        auto clipNode = new osg::ClipNode;
        auto clipPlane = new osg::ClipPlane;
        clipPlane->setClipPlaneNum(1);
        clipPlane->setClipPlane(0, 0, -1, top);
        clipNode->addClipPlane(clipPlane);

        _refractRttCamera->addChild(clipNode);

        auto ss = _refractRttCamera->getOrCreateStateSet();
        ss->setMode(
            GL_CLIP_PLANE1, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

        _refractRttCamera->addChild(_sceneRoot);
    }

    // pool
    _pool = osg::createTexturedQuadGeometry(osg::Vec3(-radius, -radius, top),
        osg::Vec3(radius * 2, 0, 0), osg::Vec3(0, radius * 2, 0));
    _pool->setName("Pool");
    _pool->setNodeMask(nb_visible);

    _sceneRoot->addChild(_pool);

    auto ss = _pool->getOrCreateStateSet();
    ss->setAttributeAndModes(createProgram("shader/pool.vert", "shader/pool.frag"));
    ss->setAttributeAndModes(
        new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    _dudvMap = new osg::Texture2D(osgDB::readImageFile("texture/dudv_map.png"));
    _dudvMap->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    _dudvMap->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    _normalMap = new osg::Texture2D(osgDB::readImageFile("texture/normal_map.png"));
    _normalMap->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    _normalMap->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);

    ss->setTextureAttributeAndModes(0, _reflectMap);
    ss->setTextureAttributeAndModes(1, _refractMap);
    ss->setTextureAttributeAndModes(2, _depthMap);
    ss->setTextureAttributeAndModes(3, _dudvMap);
    ss->setTextureAttributeAndModes(4, _normalMap);

    ss->addUniform(new osg::Uniform("reflect_map", 0));
    ss->addUniform(new osg::Uniform("refract_map", 1));
    ss->addUniform(new osg::Uniform("depth_map", 2));
    ss->addUniform(new osg::Uniform("dudv_map", 3));
    ss->addUniform(new osg::Uniform("normal_map", 4));

    auto material = new osg::Material;
    material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.6, 0.6, 0.6, 1));
    material->setShininess(osg::Material::FRONT_AND_BACK, 50);
    ss->setAttributeAndModes(material);

    // add some fishes
    auto root = new osg::Group;
    root->setNodeMask(nb_fish);
    root->setCullCallback(new PoolCreatureCullCallback);
    root->setName("PoolCreatures");
    _sceneRoot->addChild(root);

    // load model, scale it to certain size
    auto model = osgDB::readNodeFile("model/fish.osgt");
    auto size = sgc.getFloat("fish.size");
    auto& bs = model->getBound();
    auto scale = size / bs.radius();
    auto frame = new osg::MatrixTransform;
    frame->setMatrix(osg::Matrix::scale(osg::Vec3(scale, scale, scale)));
    frame->addChild(model);
    _fish = frame;

    auto count = sgc.getInt("fish.count");
    auto poolBottomRadius = sgc.getFloat("pool.bottomRadius");
    auto poolDepth = sgc.getFloat("pool.depth");
    for (auto i = 0; i < count; ++i)
    {
        auto pos = osg::Vec3(
            diskRand(poolBottomRadius * 0.9), top - linearRand(5, poolDepth - 16));
        auto fish = new osg::MatrixTransform;
        fish->setName("Fish" + std::to_string(i));
        fish->setMatrix(osg::Matrix::translate(pos));
        fish->addChild(_fish);
        fish->addUpdateCallback(new FishUpdater());
        root->addChild(fish);
    }
}

void Game::createMeadow()
{
    auto root = new osg::Group;
    root->setName("Meadow");
    root->setNodeMask(nb_unreal_object);

    auto origin = osg::Vec2(-_sceneRadius, -_sceneRadius);
    auto grassSize = sgc.getFloat("meadow.grass.size");
    auto step = sgc.getFloat("meadow.grass.step") * grassSize;
    auto numGroups = sgc.getInt("meadow.numGroupsPerRow");

    // create grasses
    int cols = _sceneRadius * 2 / step;
    int rows = _sceneRadius * 2 / step;

    auto poolRadius = sgc.getFloat("pool.radius");
    auto poolRadius2 = poolRadius * poolRadius;
    auto pos = osg::Vec2();
    auto count = 0;

    auto geometry = new osg::Geometry;
    geometry->setName("Grass");

    auto vertices = new osg::Vec3Array();
    vertices->reserve(cols * rows);

    auto hsize = grassSize * 0.5;

    for (auto i = 1; i < cols - 1; ++i)
    {
        pos.y() = i % 2 ? 0 : 0.5 * step;
        pos.x() += step;

        for (auto j = 1; j < rows - 1; ++j)
        {
            pos.y() += step;

            auto p = origin + pos + toy::diskRand(step * 0.5);
            if (p.length2() < poolRadius2)
            {
                continue;
            }

            ++count;
            vertices->push_back(getTerrainPoint(p.x(), p.y()));
        }
    }

    geometry->setVertexArray(vertices);
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size()));
    geometry->setDataVariance(osg::Object::DYNAMIC);
    geometry->setUseDisplayList(false);
    geometry->setUseVertexArrayObject(false);

    // sort by depth
    auto sortByDepth = osgf::createCallback([](osg::Object* obj, osg::Object* data) {
        auto cv = data->asNodeVisitor()->asCullVisitor();
        if (cv->getCurrentCamera() != sgg.getMainCamera())
        {
            return;
        }

        auto lookVector = cv->getLookVectorLocal();
        auto eyeLocal = cv->getEyeLocal();

        auto geom = obj->asNode()->asGeometry();
        auto vertices = static_cast<osg::Vec3Array*>(geom->getVertexArray());

        std::sort(vertices->begin(), vertices->end(),
            [&](const osg::Vec3& v0, const osg::Vec3& v1) -> bool {
                return (v0 - eyeLocal) * lookVector > (v1 - eyeLocal) * lookVector;
            });

        vertices->dirty();
        // no dirty bound and gl object
    });
    geometry->addCullCallback(sortByDepth);

    root->addChild(geometry);

    OSG_NOTICE << "Create " << count << " grasses" << std::endl;

    auto ss = root->getOrCreateStateSet();

    auto texture = new osg::Texture2D(osgDB::readImageFile("texture/grass0.png"));
    texture->setResizeNonPowerOfTwoHint(false);
    ss->setTextureAttributeAndModes(0, texture);

    auto material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8, 0.8, 0.8, 0.8));

    ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    ss->setAttributeAndModes(material);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    ss->setAttributeAndModes(
        new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));

    auto prg = createProgram("shader/grass.vert", "shader/grass.geom", "shader/grass.frag",
        GL_POINTS, GL_TRIANGLE_STRIP, 18);
    ss->setAttributeAndModes(prg);

    ss->addUniform(new osg::Uniform("size", grassSize));
    ss->addUniform(new osg::Uniform("viewMatrix", osg::Matrixf()));
    ss->addUniform(new osg::Uniform(osg::Uniform::FLOAT_VEC4, "explosions", 16));
    auto maxExplosions = sgc.getInt("meadow.maxExplosions");
    auto explosionRadius = sgc.getInt("lightning.explosionRadius");
    ss->setDefine("MAX_EXPLOSIONS", std::to_string(maxExplosions));
    ss->setDefine("EXPLOSION_RADIUS", std::to_string(explosionRadius));

    _explosions.assign(maxExplosions, osg::Vec4());
    // update explosion uniform.
    // This cull callback is called by both main camera and reflectRttCamera, it only update
    // uniform for main camera, which means you won't see grass animation in the pool, this
    // shouldn't matter, as you can only see only a few of grasses anyway. If you really
    // want grass animation, you must create a new node with a new stateset to pass
    // explosion uniform.
    auto explosionUniform = ss->getUniform("explosions");
    root->addCullCallback(osgf::createCallback([=](osg::Object* obj, osg::Object* data) {
        auto visitor = data->asNodeVisitor()->asCullVisitor();
        if (visitor->getCurrentCamera() == getMainCamera())
        {
            for (auto i = 0; i < _explosions.size(); ++i)
            {
                explosionUniform->setElement(i, _explosions[i]);
            }
        }
    }));

    _winds.resize(sgc.getInt("wind.count"));
    root->addUpdateCallback(osgf::createCallback([=](osg::Object* obj, osg::Object* data) {
        for (auto i = 0; i < _winds.size(); ++i)
        {
            _winds[i].update(sgg.getDeltaTime());
            _winds[i].updateUniform(*ss, i);
        }
    }));

    _sceneRoot->addChild(root);
}

void Game::createBurrows()
{
    auto burrowRadius = sgc.getFloat("burrow.radius");
    auto spawnRadius = sgc.getFloat("burrow.spawnRadius") * _sceneRadius;
    auto spawnDistance = sgc.getFloat("burrow.spawnDistance");
    auto points = poissonDiskSample(osg::Vec2(-spawnRadius, -spawnRadius),
        osg::Vec2(spawnRadius, spawnRadius), spawnDistance, 32);

    auto poolRadius = sgc.getFloat("pool.radius");
    for (auto& p: points)
    {
        if (p.length() - burrowRadius < poolRadius)
        {
            continue;
        }

        auto point = getTerrainPoint(p.x(), p.y());
        auto normal = getTerrainNormal(p.x(), p.y());
        normal.normalize();
        auto burrow = createBurrow(point, normal);
        // _sceneRoot->addChild(burrow.node);
        _burrowList.push_back(burrow);
    }
}

Burrow Game::createBurrow(const osg::Vec3& pos, const osg::Vec3& normal)
{
    static osg::ref_ptr<osg::ShapeDrawable> graph;
    if (!graph)
    {
        graph = new osg::ShapeDrawable(new osg::Cylinder(
            osg::Vec3(), sgc.getFloat("burrow.radius"), sgc.getFloat("burrow.height")));
        graph->setName("Burrow");
        graph->setColor(osg::Vec4(0.2f, 0.2f, 0.2f, 0.2f));

        auto ss = graph->getOrCreateStateSet();
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    }

    auto frame = new osg::MatrixTransform;
    frame->setNodeMask(nb_real_object);
    osg::Matrix m = osg::Matrix::rotate(osg::Z_AXIS, normal);
    m.postMultTranslate(pos);
    frame->setMatrix(m);
    frame->addChild(graph);
    return Burrow{false, normal, frame};
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

class TimerBarUpdater : public osg::StateSet::Callback
{
public:
    TimerBarUpdater(osg::Uniform* uniform) : _uniform(uniform) {}

private:
    void operator()(osg::StateSet*, osg::NodeVisitor*) override
    {
        _uniform->set(sgg.getPercentTime());
    }
    osg::Uniform* _uniform = 0;
};

osg::Node* Game::createUI()
{
    // Most thing will be resized in the end of this method

    // create timer text, bar
    {
        _timerBar = osg::createTexturedQuadGeometry(
            osg::Vec3(), osg::Vec3(1, 0, 0), osg::Vec3(0, 1, 0));

        auto ss = _timerBar->getOrCreateStateSet();

        auto prg = createProgram("shader/timer_bar.frag", osg::Shader::FRAGMENT);
        ss->setAttributeAndModes(prg);

        ss->addUniform(new osg::Uniform("size", osg::Vec2(1, 1)));
        auto percentUniform = new osg::Uniform("percent", 1.0f);
        ss->addUniform(percentUniform);
        ss->setUpdateCallback(new TimerBarUpdater(percentUniform));
    }

    _score = 0;
    _scoreText = createText("Score", "0", 18, osg::Vec3());
    _timerText = createText("Timer", "30", 18, osg::Vec3());
    _msg = createText("Msg", "Press r to start new game.", 18, osg::Vec3());

    auto root = new osg::Group;
    root->addChild(_scoreText);
    root->addChild(_msg);
    root->addChild(_timerText);
    root->addChild(_timerBar);
    root->setNodeMask(nb_ui);

    hide(_scoreText);
    hide(_timerText);
    hide(_timerBar);
    hide(_timerText);

    // create cursor
    {
        _cursorFrame = new osg::MatrixTransform;
        _cursorFrame->setName("CursorFrame");
        root->addChild(_cursorFrame);

        // draw lightning cursorFrame, it's 3 + 3 vertices, symmetry by 0.5, 0.5
        auto texcoords = new osg::Vec2Array();
        auto tc0 = osg::Vec2(0.5, 1);
        auto tc1 = osg::Vec2(0.25, 0.5);
        auto tc2 = osg::Vec2(0.55, 0.4);
        auto tc3 = osg::Vec2(1 - tc0.x(), 1 - tc0.y());
        auto tc4 = osg::Vec2(1 - tc1.x(), 1 - tc1.y());
        auto tc5 = osg::Vec2(1 - tc2.x(), 1 - tc2.y());
        texcoords->push_back(tc0);
        texcoords->push_back(tc1);
        texcoords->push_back(tc2);
        texcoords->push_back(tc3);
        texcoords->push_back(tc4);
        texcoords->push_back(tc5);
        texcoords->push_back(tc0);

        // Tesselator doesn't support Vec2Array
        auto cursorSize = sgc.getFloat("ui.cursor.size");

        auto vertices = new osg::Vec3Array();
        vertices->push_back(osg::Vec3((tc0 - osg::Vec2(0.5, 0.5)) * cursorSize, 0));
        vertices->push_back(osg::Vec3((tc1 - osg::Vec2(0.5, 0.5)) * cursorSize, 0));
        vertices->push_back(osg::Vec3((tc2 - osg::Vec2(0.5, 0.5)) * cursorSize, 0));
        vertices->push_back(osg::Vec3((tc3 - osg::Vec2(0.5, 0.5)) * cursorSize, 0));
        vertices->push_back(osg::Vec3((tc4 - osg::Vec2(0.5, 0.5)) * cursorSize, 0));
        vertices->push_back(osg::Vec3((tc5 - osg::Vec2(0.5, 0.5)) * cursorSize, 0));
        vertices->push_back(osg::Vec3((tc0 - osg::Vec2(0.5, 0.5)) * cursorSize, 0));

        // body
        {
            _cursorGeom = new osg::Geometry;
            _cursorGeom->setName("Cursor");

            _cursorGeom->setVertexArray(vertices);
            _cursorGeom->setTexCoordArray(0, texcoords);

            _cursorGeom->addPrimitiveSet(
                new osg::DrawArrays(GL_POLYGON, 0, vertices->size()));

            osgUtil::Tessellator ts;
            ts.setTessellationType(osgUtil::Tessellator::TESS_TYPE_POLYGONS);
            ts.setWindingType(osgUtil::Tessellator::TESS_WINDING_POSITIVE);
            ts.retessellatePolygons(*_cursorGeom);

            auto ss = _cursorGeom->getOrCreateStateSet();
            ss->setAttributeAndModes(
                createProgram("shader/cursor.frag", osg::Shader::FRAGMENT));
            ss->addUniform(new osg::Uniform("flash", 0));

            _cursorFrame->addChild(_cursorGeom);
        }

        // outline
        {
            auto outline = new osg::Geometry;
            outline->setName("CursorOutline");
            outline->setVertexArray(vertices);

            auto colors = new osg::Vec4Array(osg::Array::BIND_OVERALL);
            colors->push_back(osg::Vec4(0.75, 0, 0, 1));
            outline->setColorArray(colors);

            outline->addPrimitiveSet(
                new osg::DrawArrays(GL_LINE_STRIP, 0, vertices->size()));
            osgf::addLineSmooth(*outline->getOrCreateStateSet(), 1);

            _cursorFrame->addChild(outline);
        }
    }

    return root;
}

void Game::createStarfield()
{
    auto root = new osg::Group;
    root->setNodeMask(nb_unreal_object);
    root->setName("Starfield");

    auto rootSS = root->getOrCreateStateSet();

    // render it between opaque and transparent
    rootSS->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 1.0f, 1.0f, false));
    rootSS->setRenderBinDetails(5, "RenderBin");
    rootSS->setMode(GL_BLEND, osg::StateAttribute::ON);

    auto projNode = new osg::Projection(createDefaultProjectionMatrix(0.1, 10000));
    root->addChild(projNode);

    // add moon
    auto radius = sgc.getFloat("starfield.radius");
    auto radius2 = radius * radius;
    auto moonPos = sgc.getVec3("starfield.moon.pos");
    moonPos.normalize();
    moonPos *= radius;
    {
        auto moon = osgf::createPoints({moonPos});
        moon->setName("Moon");
        moon->setCullingActive(false);
        moon->setComputeBoundingBoxCallback(
            static_cast<osg::Drawable::ComputeBoundingBoxCallback*>(
                osgf::getInvalidComputeBoundingBoxCallback()));

        auto ss = moon->getOrCreateStateSet();
        ss->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
        ss->setTextureAttributeAndModes(0, new osg::PointSprite());
        ss->setAttributeAndModes(new osg::BlendFunc(
            osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));

        static auto program = createProgram("shader/moon.vert", "shader/moon.frag");
        ss->setAttributeAndModes(program);

        projNode->addChild(moon);

        // add moon as global sky light
        auto ls = new osg::LightSource;
        auto light = ls->getLight();
        light->setLightNum(1);
        light->setPosition(osg::Vec4(moonPos, 0));
        light->setAmbient(sgc.getVec4("starfield.moon.ambient"));
        light->setSpecular(sgc.getVec4("starfield.moon.specular"));

        _sceneRoot->addChild(ls);
        ls->setStateSetModes(*_sceneRoot->getOrCreateStateSet(), osg::StateAttribute::ON);
    }

    // add stars
    {
        auto stars = new osg::Geometry;
        auto vertices = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
        auto numStars = sgc.getInt("scene.numStars");
        vertices->reserve(numStars);
        for (auto i = 0; i < numStars; ++i)
        {
            auto pos = sphericalRand(radius);
            if ((pos - moonPos).length2() < radius2 * 0.01f)
            {
                continue;
            }
            vertices->push_back(osg::Vec4(pos, linearRand(0.0f, 0.5f)));
        }

        stars->setVertexArray(vertices);
        stars->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size()));
        stars->setCullingActive(false);
        stars->setComputeBoundingBoxCallback(
            static_cast<osg::Drawable::ComputeBoundingBoxCallback*>(
                osgf::getInvalidComputeBoundingBoxCallback()));

        auto ss = stars->getOrCreateStateSet();
        ss->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
        ss->setTextureAttributeAndModes(0, new osg::PointSprite());
        ss->setAttributeAndModes(new osg::BlendFunc(
            osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));

        static auto program = createProgram("shader/star.vert", "shader/star.frag");
        ss->setAttributeAndModes(program);

        projNode->addChild(stars);
    }

    _sceneRoot->addChild(root);
}

void Game::playWhackAnimation(const osg::Vec3& pos)
{
    auto l = new Lightning;
    l->setNodeMask(nb_unreal_object);
    l->setBillboardWidth(sgc.getFloat("lightning.billboardWidth"));
    l->setMaxJitter(0.37);
    l->add("jjjjjj", pos + osg::Vec3(diskRand(5), 168), pos);

    auto startExponent = 0.5f;
    auto exponent = new osg::Uniform("exponent", startExponent);
    auto ss = l->getOrCreateStateSet();
    ss->addUniform(exponent);

    _sceneRoot->addChild(l);
    l->addUpdateCallback(osgf::createCallback([=](osg::Object* object, osg::Object* data) {
        float v;
        exponent->get(v);
        exponent->set(v -= startExponent * sgg.getDeltaTime());
    }));
    _sceneRoot->addUpdateCallback(osgf::createTimerRemoveNodeUpdateCallback(1, l));
}

void Game::popScore(const osg::Vec3& pos, int score)
{
    auto text = createText("PopScore", std::to_string(score), 22, pos);
    text->setNodeMask(nb_visible);
    text->setAlignment(osgText::Text::CENTER_CENTER);
    text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
    text->setColor(osg::Vec4(1, 0, 0, 1));
    text->setAutoRotateToScreen(true);

    auto ss = text->getOrCreateStateSet();
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    text->addUpdateCallback(osgf::createCallback([=](auto obj, auto data) {
        text->setPosition(text->getPosition() + osg::Vec3(0, 0, 10) * sgg.getDeltaTime());
    }));

    _sceneRoot->addChild(text);
    _sceneRoot->addUpdateCallback(osgf::createTimerRemoveNodeUpdateCallback(1, text));
}

void Game::explode(const osg::Vec3& pos)
{
    auto iter = std::find_if(
        _explosions.begin(), _explosions.end(), [](auto& v) { return v.w() <= 0; });
    if (iter == _explosions.end())
    {
        OSG_NOTICE << "Running out of explosions" << std::endl;
        return;
    }

    auto force = sgc.getFloat("lightning.explosionForce");
    auto duration = sgc.getFloat("lightning.explosionDuration");
    *iter = osg::Vec4(pos, force);
    auto index = std::distance(_explosions.begin(), iter);
    auto explosionUpdater = osgf::createCallback([=](auto obj, auto data) {
        _explosions[index].w() -= sgg.getDeltaTime() * force / duration;
    });
    _sceneRoot->addUpdateCallback(explosionUpdater);
    _sceneRoot->addUpdateCallback(
        osgf::createTimerUpdateCallback(duration, [=](auto obj, auto data) {
            _sceneRoot->removeUpdateCallback(explosionUpdater);
            _explosions[index].w() = 0;
        }));
}

class StartAnimationUpdater : public osg::Callback
{
public:
    StartAnimationUpdater(Mole* mole) : _mole(mole) {}

    bool run(osg::Object* object, osg::Object* data) override
    {
        auto m = _mole->getMatrix();
        auto pos = m.getTrans();

        if (pos.x() > 5)
        {
            if (!_mole->getKicked())
            {
                _mole->setKicked(true);

                auto l = new Lightning;
                l->setBillboardWidth(10);
                l->setMaxJitter(0.37);
                l->add("jjjjjj", pos + osg::Vec3(diskRand(5), 100), pos);

                playSound(_mole, new ALSource(osgDB::readALBufferFile("sound/hit.wav")));

                sgg.getSceneRoot()->addChild(l);
            }
        }
        else
        {
            auto dt = sgg.getDeltaTime();
            m.postMultTranslate(osg::Vec3(1, 0, 0) * dt);
            _mole->setMatrix(m);
        }

        return traverse(object, data);
    }

private:
    Mole* _mole = 0;
};

void Game::createStartAnimation()
{
    auto root = new osg::Group;
    root->setName("StartAnimation");

    auto mole = new Mole(0);
    root->addChild(mole);
    _sceneRoot->addUpdateCallback(new StartAnimationUpdater(mole));

    _sceneRoot->addChild(root);
}

}  // namespace toy
