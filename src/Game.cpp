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
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/Layer>
#include <osgTerrain/Locator>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgText/Text>
#include <osgUtil/PerlinNoise>
#include <osgUtil/TangentSpaceGenerator>
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
#include <ToyShadowMap.h>

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
    if (_time < 0)
    {
        mutate();
    }
    else if (_time >= _duration * 0.8)
    {
        // fade in
        _strength = mix(0.0f, _amplitude, (_duration - _time) / (_duration * 0.2f));
    }
    else if (_time <= _duration * 0.2)
    {
        // fade out
        _strength = mix(0.0f, _amplitude, _time / (_duration * 0.2f));
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
    _length = clamp(gaussRand(sgc.getVec2("wind.length.gauss")), 0.0f, 99999.f);
    _duration = clamp(gaussRand(sgc.getVec2("wind.duration")), 0.0f, 99999.f);
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
        _timer -= _deltaTime;

        if (_timerText)
        {
            std::stringstream ss;
            ss << std::setprecision(1) << std::fixed << std::showpoint << _timer;
            _timerText->setText(ss.str());
        }

        if (_timer <= 0)
        {
            timeout();
        }
    }

    if (_status != gs_init && _shadowMap)
    {
        auto ghostManipulator =
            dynamic_cast<GhostManipulator*>(_manipulator->getMatrixManipulatorWithIndex(1));
        _shadowMap->setCenter(ghostManipulator->getEye());
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
    sgc.reload();

    if (sgc.getBool("scene.shadow"))
    {
        _shadowedScene = new osgShadow::ShadowedScene();
        _shadowedScene->setName("ShadowedScene");
        // shadow map doesn't support receive shadow mask
        // _shadowedScene->setReceivesShadowTraversalMask(nb_receive_shadow);
        _shadowedScene->setCastsShadowTraversalMask(nb_cast_shadow);

        _shadowMap = new ToyShadowMap;
        _shadowedScene->setShadowTechnique(_shadowMap);
        _shadowMap->setTextureSize(sgc.getVec2s("scene.shadow.texture.size"));
        _shadowMap->addShader(osgDB::readShaderFile("shader/shadow.frag"));

        _root->replaceChild(_sceneRoot, _shadowedScene);
        _sceneRoot = _shadowedScene;
    }

    auto camera = getMainCamera();
    camera->setClearColor(osg::Vec4(0.1, 0.1, 0.1, 0.1));

    if (sgc.getBool("scene.ui"))
    {
        createUI();
    }

    _root->addUpdateCallback(this);
    _root->addEventCallback(new GameEventHandler);

    osgUtil::PerlinNoise pn;
    _noiseTexture3D = pn.create3DNoiseTexture(sgc.getInt("starfield.sky.texture.size"));

    createStartAnimation();
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
    mole->setNodeMask(nb_real_object | nb_cast_shadow);
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
    if (_scoreText)
    {
        _scoreText->setText(std::to_string(_score));
    }
}

void Game::restart()
{
    OSG_NOTICE << "Restart game" << std::endl;

    sgc.reload();

    clear();

    // start new game
    _status = gs_running;

    if (sgc.getBool("scene.ui"))
        resetUI();

    // cache some frequently used settings
    _sceneHeight = sgc.getFloat("scene.height");
    _popRate = sgc.getFloat("mole.popRate");

    createLights();

    if (_shadowMap)
    {
        _shadowMap->setProjectionSize(sgc.getVec2("scene.shadow.projectionSize"));
        _shadowMap->setMainCamera(getMainCamera());
    }

    if (sgc.getBool("scene.starfield"))
        createStarfield();

    createTerrain();

    if (sgc.getBool("scene.pool"))
        createPool();

    if (sgc.getBool("scene.trees"))
        createTrees();

    if (sgc.getBool("scene.meadow"))
        createMeadow();

    createBurrows();

    setupCameraAndManipulator();

    // force a resize in the end
    auto wsize = osgq::getWindowRect(*_viewer);
    resize(wsize.z(), wsize.w());
}

void Game::timeout()
{
    _status = gs_timeout;
    if (_msg)
    {
        _msg->setNodeMask(nb_ui);
        _msg->setText("Press r to start new game.");
    }
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
    if (_uiRoot)
    {
        auto barSize = sgc.getVec2("ui.bar.size");
        barSize = osg::componentMultiply(barSize, osg::Vec2(width, height));
        auto y = sgc.getFloat("ui.bar.y");
        auto x = (width - barSize.x()) * 0.5f;

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
    }

    if (_status == gs_init)
    {
        return;
    }

    if (_reflectRttCamera)
    {
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
}

void Game::moveCursor(float x, float y)
{
    if (_cursorFrame)
    {
        _cursorFrame->setMatrix(osg::Matrix::translate(osg::Vec3(x, y, 0)));
    }
}

void Game::flashCursor(bool v)
{
    static bool lastFlash = false;
    if (lastFlash == v || !_cursorFrame)
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
    auto col = (x - _terrainOrigin.x()) / _tileSize;
    auto row = (y - _terrainOrigin.y()) / _tileSize;
    auto ndcX = col - std::floor(col);
    auto ndcY = row - std::floor(row);
    if (ndcX == 0 && col >= 1)
        --col;
    if (ndcY == 0 && row >= 1)
        --row;

    auto tile = _terrain->getTile(osgTerrain::TileID(0, col, row));
    if (!tile)
    {
        OSG_WARN << "Failed to get Terrain Tile for " << x << "," << y << std::endl;
        return osg::Vec3();
    }

    auto layer = static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());

    float h;
    if (layer->getInterpolatedValue(ndcX, ndcY, h))
        return osg::Vec3(x, y, h);
    else
    {
        OSG_WARN << "Failed to get terrain point for " << x << "," << y << std::endl;
        return osg::Vec3();
    }
}

osgTerrain::TerrainTile* Game::getTerrainTile(float x, float y)
{
    auto col = (x - _terrainOrigin.x()) / _tileSize;
    auto row = (y - _terrainOrigin.y()) / _tileSize;
    return _terrain->getTile(osgTerrain::TileID(0, col, row));
}

osg::Vec3 Game::getTerrainNormal(float x, float y)
{
    auto col = (x - _terrainOrigin.x()) / _tileSize;
    auto row = (y - _terrainOrigin.y()) / _tileSize;
    auto tile = _terrain->getTile(osgTerrain::TileID(0, col, row));
    if (!tile)
    {
        OSG_WARN << "Failed to get Terrain Tile for " << x << "," << y << std::endl;
        return osg::Vec3();
    }

    auto layer = static_cast<osgTerrain::HeightFieldLayer*>(tile->getElevationLayer());
    auto heightField = layer->getHeightField();

    auto ndcX = col - std::floor(col);
    auto ndcY = row - std::floor(row);
    auto i = std::round(heightField->getNumColumns() * ndcX);
    auto j = std::round(heightField->getNumRows() * ndcY);
    return heightField->getNormal(i, j);
}

Game::Game() {}

Game::~Game() {}

void Game::clear()
{
    // clear resource observer, but leave static ones
#ifdef DEBUG
    _observer->clear();
    auto lprg = Lightning::getBillboardProgram();
    if (lprg)
    {
        _observer->addResource(*lprg);
    }

    if (_shadowMap)
    {
        _observer->addResource(*_shadowMap->getProgram());
    }

#endif

    _sceneRoot->removeChild(0, _sceneRoot->getNumChildren());
    _sceneRoot->setUpdateCallback(0);
    _sceneRoot->setCullCallback(0);
    _sceneRoot->setEventCallback(0);

    _removeMoleCallbacks.clear();
}

void Game::resetUI()
{
    if (!_uiRoot)
    {
        createUI();
    }

    _score = 0;
    _scoreText->setText("0");

    hide(_msg);
    _timer = 60;
    _totalTime = 60;
    _timerText->setText(std::to_string(_timer));

    _scoreText->setNodeMask(nb_ui);
    _timerText->setNodeMask(nb_ui);
    _timerBar->setNodeMask(nb_ui);
}

void Game::createTerrain()
{
    // Perlin noised generated height field terrain. A hole will be digged in center as
    // pool.

    auto colorLayer =
        new osgTerrain::ImageLayer(osgDB::readImageFile("texture/ground.jpg"));
    _terrain = new osgTerrain::Terrain;
    _terrain->setNodeMask(nb_terrain);

    _tileCount = sgc.getInt("terrain.tile.count");
    _tileSize = sgc.getFloat("terrain.tile.size");
    // make sure no scale happens between heightfiled and heightfield layer, otherwise
    // HeightField::getNormal will break
    auto tileRows = sgc.getInt("terrain.tile.rows");
    auto tileCols = sgc.getInt("terrain.tile.cols");
    auto xInterval = _tileSize / tileCols;
    auto yInterval = _tileSize / tileRows;
    auto origin = osg::Vec2(-_tileSize * _tileCount * 0.5f, -_tileSize * _tileCount * 0.5f);
    _terrainOrigin = origin;
    _sceneRadius = -origin.x();

    osgUtil::PerlinNoise pn;
    pn.SetNoiseFrequency(64);
    // Perlin noise used fixed seed 30757, we random the start coordinates to random the
    // terrain.
    auto startCoord = osg::Vec2(linearRand(-100.0f, 100.0f), linearRand(-100.0f, 100.0f));
    auto poolRadius = sgc.getFloat("pool.radius");
    auto poolBottomRadius = sgc.getFloat("pool.bottomRadius");
    auto poolDepth = sgc.getFloat("pool.depth");

    auto yStep = 0.03;
    auto xStep = 0.03;

    // create n*n TerrainTile. Each TerrainTile use HeightFieldLayer as elevation layer.
    for (auto i = 0; i < _tileCount; ++i)
    {
        for (auto j = 0; j < _tileCount; ++j)
        {
            // populate HeightField
            auto heightField = new osg::HeightField;
            heightField->allocate(tileRows, tileCols);
            heightField->setXInterval(xInterval);
            heightField->setYInterval(yInterval);
            heightField->setOrigin(
                osg::Vec3(_tileSize * i + origin.x(), _tileSize * j + origin.y(), 0));

            auto tileCoord = startCoord + osg::Vec2(xStep * (tileCols - 1) * i,
                                              yStep * (tileRows - 1) * j);
            auto coord = tileCoord - osg::Vec2(xStep, yStep);
            for (int y = 0; y < tileRows; ++y)
            {
                coord[1] += yStep;
                coord[0] = tileCoord.x();

                for (int x = 0; x < tileCols; ++x)
                {
                    coord[0] += xStep;
                    auto h = pn.PerlinNoise2D(coord[0], coord[1], 2, 2, 3) * _sceneHeight;

                    // dig pool
                    auto vertex = heightField->getVertex(x, y);
                    auto l = osg::Vec2(vertex.x(), vertex.y()).length();
                    if (l < poolRadius)
                    {
                        h -= mix(0.0f, poolDepth,
                            1.0f - smoothstep(poolBottomRadius, poolRadius, l));
                    }

                    heightField->setHeight(x, y, h);
                }
            }

            // create TerrainTile
            auto locator = new osgTerrain::Locator;
            locator->setCoordinateSystemType(osgTerrain::Locator::GEOGRAPHIC);
            auto tileWorldOrigin = origin + osg::Vec2(_tileSize * i, _tileSize * j);
            locator->setTransformAsExtents(tileWorldOrigin.x(), tileWorldOrigin.y(),
                tileWorldOrigin.x() + _tileSize, tileWorldOrigin.y() + _tileSize);

            auto layer = new osgTerrain::HeightFieldLayer(heightField);
            layer->setLocator(locator);

            auto tile = new osgTerrain::TerrainTile;
            tile->setName("Tile" + std::to_string(i) + "," + std::to_string(j));
            tile->setTerrainTechnique(new osgTerrain::GeometryTechnique);
            tile->setTileID(osgTerrain::TileID(0, i, j));
            tile->setElevationLayer(layer);
            tile->setColorLayer(0, colorLayer);

            tile->setTerrain(_terrain);
            _terrain->addChild(tile);
        }
    }

    auto ss = _terrain->getOrCreateStateSet();
    auto material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.75, 0.75, 0.75, 1));
    ss->setAttributeAndModes(material);
    _sceneRoot->addChild(_terrain);

    OSG_NOTICE << "Create " << _tileCount << "x" << _tileCount << " terrain tiles"
               << std::endl;
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
    // Pool is a rect textured with reflect texture and refract texture. Rtt camera is only
    // accepted if pool pass cull test. Global clip plane 0 is used to clip above pool,
    // Global clip plane 1 is ued to clip below pool. No shadow in rtt. This pool is adapted
    // from
    // https://www.youtube.com/watch?v=HusvGeEDU_U&list=PLRIWtICgwaX23jiqVByUs0bqhnalNTNZh&index=1

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

        // require stencil for outline
        _reflectRttCamera->attach(
            osg::Camera::PACKED_DEPTH_STENCIL_BUFFER, GL_DEPTH_STENCIL_EXT);
        _reflectRttCamera->setCullMask(nb_above_waterline);
        _reflectRttCamera->setClearMask(
            GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        // flip scene by pool plane
        auto m = osg::Matrix::translate(osg::Vec3(0, 0, top));
        m.preMultScale(osg::Vec3(1, 1, -1));
        m.preMultTranslate(osg::Vec3(0, 0, -top));
        _reflectRttCamera->setViewMatrix(m);

        // clip above pool
        auto clipNode = new osg::ClipNode;
        auto clipPlane = new osg::ClipPlane;
        clipPlane->setClipPlaneNum(0);
        clipPlane->setClipPlane(0, 0, 1, -top);
        clipNode->addClipPlane(clipPlane);

        _reflectRttCamera->addChild(clipNode);

        auto ss = _reflectRttCamera->getOrCreateStateSet();
        ss->setMode(
            GL_CLIP_PLANE0, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        ss->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        ss->setDefine("SHADOWED_SCENE", "0");

        _reflectRttCamera->addChild(_sceneRoot);
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

        // depth map is used to calculate pool depth in frag shader(pool bottom to pool top,
        // in camera -z direction)
        _refractRttCamera->attach(osg::Camera::DEPTH_BUFFER, _depthMap);
        _refractRttCamera->setCullMask(nb_below_waterline);
        _refractRttCamera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        // clip below pool
        auto clipNode = new osg::ClipNode;
        auto clipPlane = new osg::ClipPlane;
        clipPlane->setClipPlaneNum(1);
        clipPlane->setClipPlane(0, 0, -1, top);
        clipNode->addClipPlane(clipPlane);

        _refractRttCamera->addChild(clipNode);

        auto ss = _refractRttCamera->getOrCreateStateSet();
        ss->setMode(
            GL_CLIP_PLANE1, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        // Use fresh new program state, avoid shadow program from ShadowMap
        ss->setDefine("SHADOWED_SCENE", "0");

        _refractRttCamera->addChild(_sceneRoot);
    }

    // add pool. The pool it self should only be rended by main camera.
    _pool = osg::createTexturedQuadGeometry(osg::Vec3(-radius, -radius, top),
        osg::Vec3(radius * 2, 0, 0), osg::Vec3(0, radius * 2, 0));
    _pool->setName("Pool");
    _pool->setNodeMask(nb_visible);
    _pool->addCullCallback(osgf::createCallback([=](osg::Object* obj, osg::Object* data) {
        auto visitor = data->asNodeVisitor()->asCullVisitor();
        // unlike other nodes, drawable cull callback is always called, it's done before
        // cull test.
        if (!visitor->isCulled(*obj->asDrawable()))
        {
            // Note, current StateSet is inherited by rtt camera, this might cause problem.
            _reflectRttCamera->accept(*visitor);
            _refractRttCamera->accept(*visitor);
        }
    }));
    _sceneRoot->addChild(_pool);

    auto ss = _pool->getOrCreateStateSet();
    ss->setAttributeAndModes(createProgram("shader/pool.vert", "shader/pool.frag"));

    // alpha blend is used to soft edge
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
    ss->setTextureAttributeAndModes(2, _refractMap);
    ss->setTextureAttributeAndModes(3, _depthMap);
    ss->setTextureAttributeAndModes(4, _dudvMap);
    ss->setTextureAttributeAndModes(5, _normalMap);

    ss->addUniform(new osg::Uniform("reflect_map", 0));
    ss->addUniform(new osg::Uniform("refract_map", 2));
    ss->addUniform(new osg::Uniform("depth_map", 3));
    ss->addUniform(new osg::Uniform("dudv_map", 4));
    ss->addUniform(new osg::Uniform("normal_map", 5));
    ss->addUniform(new osg::Uniform("osgShadow_ambientBias", osg::Vec2(0.6, 0.4)));

    auto material = new osg::Material;
    material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.6, 0.6, 0.6, 1));
    material->setShininess(osg::Material::FRONT_AND_BACK, 50);
    ss->setAttributeAndModes(material);

    // add some swimming fishes
    auto fishRoot = new osg::Group;
    fishRoot->setNodeMask(nb_fish);
    fishRoot->setCullCallback(new PoolCreatureCullCallback);
    fishRoot->setName("PoolCreatures");
    _sceneRoot->addChild(fishRoot);

    _fish = osgf::readNodeFile("model/fish.osgt", sgc.getFloat("fish.size"));

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
        fishRoot->addChild(fish);
    }
}

void Game::createTrees()
{
    auto root = new osg::Group;
    root->setName("TreeRoot");
    root->setNodeMask(nb_unreal_object | nb_cast_shadow);

    _sceneRoot->addChild(root);

    auto tree = osgDB::readNodeFile("model/spruce.osgt");

    auto drawables = osgq::searchType<osg::Drawable>(*tree, &osg::Node::asDrawable);
    assert(drawables.size() == 2);
    auto trunk = dynamic_cast<osg::Geometry*>(drawables[0].back());
    auto leaf = dynamic_cast<osg::Geometry*>(drawables[1].back());

    // generate tangent array. You shoud not use cross in treeit, all cross share the same
    // normal, it'll break TangentSpaceGenerator.
    osg::ref_ptr<osgUtil::TangentSpaceGenerator> tsg = new osgUtil::TangentSpaceGenerator;
    tsg->generate(trunk);
    trunk->setTexCoordArray(1, tsg->getTangentArray());

    tsg->generate(leaf);
    leaf->setTexCoordArray(1, tsg->getTangentArray());

    // render with program, add normal map
    auto trunkStateSet = trunk->getOrCreateStateSet();
    // Some triangle is not in the right order
    trunkStateSet->setMode(
        GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    trunkStateSet->setAttributeAndModes(
        sgg.createProgram("shader/tree.vert", "shader/trunk.frag"));
    auto trunkNormalMap = new osg::Texture2D(osgDB::readImageFile("texture/bark07_n.png"));
    trunkNormalMap->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    trunkNormalMap->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    trunkStateSet->setTextureAttributeAndModes(1, trunkNormalMap);
    trunkStateSet->addUniform(new osg::Uniform("diffuse_map", 0));
    trunkStateSet->addUniform(new osg::Uniform("normal_map", 1));

    auto leafStateSet = leaf->getOrCreateStateSet();
    leafStateSet->setMode(
        GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    leafStateSet->setAttributeAndModes(
        sgg.createProgram("shader/tree.vert", "shader/leaf.frag"));
    leafStateSet->setDefine("FLUTTER");
    leafStateSet->addUniform(new osg::Uniform("diffuse_map", 0));
    leafStateSet->addUniform(new osg::Uniform("normal_map", 1));
    auto leafNormalMap =
        new osg::Texture2D(osgDB::readImageFile("texture/spruce_branch_n.png"));
    leafNormalMap->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    leafNormalMap->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    leafStateSet->setTextureAttributeAndModes(1, leafNormalMap);
    leafStateSet->addUniform(new osg::Uniform("diffuse_map", 0));
    leafStateSet->addUniform(new osg::Uniform("normal_map", 1));

    // generate trees
    auto poolRadius = sgc.getFloat("pool.radius");
    auto poolRadius2 = poolRadius * poolRadius;

    auto points = poissonDiskSample(-osg::Vec2(_sceneRadius, _sceneRadius),
        osg::Vec2(_sceneRadius, _sceneRadius), sgc.getFloat("tree.interval"), 32);
    for (auto& p: points)
    {
        if (p.length2() < poolRadius2)
        {
            continue;
        }
        auto frame = new osg::MatrixTransform;
        auto m = osg::Matrix::translate(getTerrainPoint(p.x(), p.y()));
        m.preMultRotate(osg::Quat(linearRand(0.0f, 2.0f * osg::PIf), osg::Z_AXIS));
        frame->setMatrix(m);
        frame->addChild(tree);
        root->addChild(frame);
    }

    // apply winds
    auto windSize = sgc.getVec4("tree.wind.size");
    auto windSpeed = sgc.getVec2("wind.speed.gauss");
    auto windAmplitude = sgc.getVec2("wind.amplitude.gauss");
    auto meanStrength = windSpeed.x() * windAmplitude.x();
    auto windSizeUniform = new osg::Uniform("wind_size", windSize);
    root->getOrCreateStateSet()->addUniform(windSizeUniform);
    root->addUpdateCallback(osgf::createCallback([=](auto* obj, auto* data) {
        auto maxStrength = 0.000001f;
        for (auto& wind: _winds)
        {
            maxStrength = std::max(wind.getSpeed() * wind.getStrength(), maxStrength);
        }
        windSizeUniform->set(osg::Vec4(windSize.x(),
            windSize.y() * meanStrength / maxStrength, windSize.z(), windSize.w()));
    }));
}

class SortByDepth : public osg::Callback
{
public:
    SortByDepth()
    {
        _maxDistance = sgc.getFloat("meadow.group.sort.maxDistance");
        _sortFrames = sgc.getFloat("meadow.group.sort.frames");
    }

    bool run(osg::Object* object, osg::Object* data) override
    {
        auto cv = data->asNodeVisitor()->asCullVisitor();
        if (cv->getCurrentCamera() != sgg.getMainCamera())
        {
            return traverse(object, data);
        }

        auto lookVector = cv->getLookVectorLocal();
        auto eyeLocal = cv->getEyeLocal();

        auto leaf = object->asNode()->asGroup();
        assert(leaf && leaf->getNumChildren() == 1);

        auto ev = eyeLocal - leaf->getBound().center();

        if (cv->getFrameStamp()->getFrameNumber() % _sortFrames != 0 &&
            ev.length() > _maxDistance)
        {
            return traverse(object, data);
        }

        auto geom = leaf->getChild(0)->asGeometry();

        auto& vertices = static_cast<osg::Vec3Array&>(*geom->getVertexArray());

        std::sort(vertices.begin(), vertices.end(),
            [&](const osg::Vec3& v0, const osg::Vec3& v1) -> bool {
                return (v0 - eyeLocal) * lookVector > (v1 - eyeLocal) * lookVector;
            });
        vertices.dirty();
        // no dirty bound and gl object
        return traverse(object, data);
    }
private:
    float _maxDistance = 1024;
    int _sortFrames = 8;
};

void Game::createMeadow()
{
    // We only store grass world position here, real grass is generate as * in geom shader.
    // The grasses will be sorted in cull callback. In order to speed up cull and sort, the
    // meadow is divied into n * n groups. All visible grasses are sorted per
    // meadow.grass.sort.frames, otherwise only grasses in maeadow.group.sort.maxDistance
    // are sorted.
    //
    // Known issue : If you walk along the group edge, the blend will be wrong.
    //
    // This grass is adapted from
    // https://developer.nvidia.com/gpugems/gpugems/part-i-natural-effects/chapter-7-rendering-countless-blades-waving-grass
    // Grass animation is adapted from
    // https://developer.nvidia.com/gpugems/gpugems/part-i-natural-effects/chapter-1-effective-water-simulation-physical-models

    auto root = new osg::Group;
    root->setNodeMask(nb_unreal_object);
    _sceneRoot->addChild(root);

    // create meadow geometry for each tile
    auto numGroups = sgc.getInt("meadow.group.count");
    auto groupSize = _sceneRadius * 2 / numGroups;

    // divide meadow
    std::map<int, osg::Geometry*> meadowMap;
    for (auto i = 0; i < numGroups; i++)
    {
        for (auto j = 0; j < numGroups; j++)
        {
            auto name = "Meadow" + std::to_string(i) + ":" + std::to_string(j);

            auto group = new osg::Group;
            group->setName(name);
            root->addChild(group);

            auto geom = new osg::Geometry;
            geom->setName(name);

            auto vertices = new osg::Vec3Array;
            geom->setVertexArray(vertices);
            geom->setUseDisplayList(false);
            geom->setUseVertexArrayObject(false);
            geom->setDataVariance(osg::Object::DYNAMIC);

            group->addChild(geom);

            meadowMap[j * numGroups + i] = geom;
        }
    }

    auto addGrass = [&](const osg::Vec2& p) {
        int i = (p.x() - _terrainOrigin.x()) / groupSize;
        int j = (p.y() - _terrainOrigin.y()) / groupSize;
        int idx = j * numGroups + i;
        auto it = meadowMap.find(idx);
        assert(it != meadowMap.end());
        auto geom = it->second;
        auto vertices = static_cast<osg::Vec3Array*>(geom->getVertexArray());
        vertices->push_back(getTerrainPoint(p.x(), p.y()));
    };

    // create grass points
    auto grassSize = sgc.getFloat("meadow.grass.size");
    auto poolRadius = sgc.getFloat("pool.radius");
    auto origin = osg::Vec2(-_sceneRadius, -_sceneRadius);
    auto step = sgc.getFloat("meadow.grass.step") * grassSize;

    int cols = _sceneRadius * 2 / step;
    int rows = _sceneRadius * 2 / step;

    auto pos = osg::Vec2(-step * 0.5f, 0.0f);
    auto minRadius = poolRadius + 0.5 * grassSize;
    auto minRadius2 = minRadius * minRadius;

    for (auto i = 0; i < cols; ++i)
    {
        // shift odd col a little bit.
        pos.y() = i % 2 ? 0 : -0.5 * step;
        pos.x() += step;

        for (auto j = 0; j < rows; ++j)
        {
            pos.y() += step;
            auto p = origin + pos;
            if (p.length2() < minRadius2)
            {
                continue;
            }
            p += toy::diskRand(step * 0.5);

            if (p.length2() < minRadius2)
            {
                p.normalize();
                p *= minRadius;
            }

            p = clamp(p, origin, -origin);
            addGrass(p);
        }
    }

    // create a circle around pool, user should not see *
    auto stepAngle = step / poolRadius;
    int stepCount = std::ceil(osg::PIf * 2 / stepAngle);
    stepAngle = osg::PIf * 2 / stepCount;
    auto r = poolRadius * 0.999f;  // slightly smaller then radius, it's used as condition
                                   // check in geom shader
    for (auto i = 0; i < stepCount; ++i)
    {
        auto angle = stepAngle * i;
        addGrass(osg::Vec2(std::cos(angle) * r, std::sin(angle) * r));

        // Fill the blank area between the circle and other grasses
        addGrass(osg::Vec2(std::cos(angle + stepAngle * 0.5f) * minRadius,
            std::sin(angle + stepAngle * 0.5f) * minRadius));
    }

    // create PrimitiveSet for each meadow
    auto count = 0;
    for (auto& pair: meadowMap)
    {
        auto geom = pair.second;
        auto vertices = geom->getVertexArray();
        geom->addPrimitiveSet(
            new osg::DrawArrays(GL_POINTS, 0, vertices->getNumElements()));
        count += vertices->getNumElements();
    }

    OSG_NOTICE << "Create " << count << " grasses" << std::endl;

    // create meadow stateset
    auto ss = root->getOrCreateStateSet();

    auto texture = new osg::Texture2D(osgDB::readImageFile("texture/grass0.png"));
    texture->setResizeNonPowerOfTwoHint(false);
    ss->setTextureAttributeAndModes(0, texture);
    ss->addUniform(new osg::Uniform("diffuse_map", 0));

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

    auto maxExplosions = sgc.getInt("meadow.maxExplosions");
    auto explosionRadius = sgc.getInt("lightning.explosionRadius");
    ss->addUniform(new osg::Uniform("size", grassSize));
    ss->addUniform(new osg::Uniform(osg::Uniform::FLOAT_VEC4, "explosions", maxExplosions));
    ss->addUniform(new osg::Uniform("pool_radius", poolRadius));
    ss->setDefine("MAX_EXPLOSIONS", std::to_string(maxExplosions));
    ss->setDefine("EXPLOSION_RADIUS", std::to_string(explosionRadius));

    // sort by depth, only done for main camera.
    _explosions.assign(maxExplosions, osg::Vec4());

    // update explosions only for main camera cull traversal
    auto explosionUniform = ss->getUniform("explosions");
    auto explosionCallback = osgf::createCallback([=](osg::Object* obj, osg::Object* data) {
        auto visitor = data->asNodeVisitor()->asCullVisitor();
        if (visitor->getCurrentCamera() == getMainCamera())
        {
            for (auto i = 0; i < _explosions.size(); ++i)
            {
                explosionUniform->setElement(i, _explosions[i]);
            }
        }
    });

    root->addCullCallback(explosionCallback);

    for (auto& pair: meadowMap)
    {
        auto leaf = pair.second->getParent(0);
        leaf->addCullCallback(new SortByDepth);
    }

    _winds.resize(sgc.getInt("wind.count"));
    _sceneRoot->addUpdateCallback(
        osgf::createCallback([=](osg::Object* obj, osg::Object* data) {
            for (auto i = 0; i < _winds.size(); ++i)
            {
                _winds[i].update(sgg.getDeltaTime());
                _winds[i].updateUniform(*ss, i);
            }
        }));
}

class MoleSpawner : public osg::Callback
{
public:
    MoleSpawner(const osg::Vec2& gaussRate) : _gaussRate(gaussRate) {}

    bool run(osg::Object* object, osg::Object* data) override
    {
        _time -= sgg.getDeltaTime();
        if (_time <= 0)
        {
            _time = clamp(gaussRand(_gaussRate), 0.001f, 2.0f);
            sgg.popMole();
        }
        return traverse(object, data);
    }

private:
    double _time = 0.2;
    osg::Vec2 _gaussRate;
};

void Game::createBurrows()
{
    _burrowList.clear();

    auto burrowRadius = sgc.getFloat("burrow.radius");
    auto spawnRadius = sgc.getFloat("burrow.spawnRadius") * _sceneRadius;
    auto spawnInterval = sgc.getFloat("burrow.spawnInterval");
    auto points = poissonDiskSample(osg::Vec2(-spawnRadius, -spawnRadius),
        osg::Vec2(spawnRadius, spawnRadius), spawnInterval, 32);

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

    auto rate = sgc.getVec2("mole.spawn.rate.gauss");
    _sceneRoot->addUpdateCallback(new MoleSpawner(rate));
}

void Game::createLights()
{
    // There are two lights in this game.
    // 1. head light, LIGHT0
    // 2. sky light, moon light, LIGHT1

    auto lm = new osg::LightModel;
    lm->setLocalViewer(true);
    lm->setAmbientIntensity(sgc.getVec4("scene.ambient"));

    auto ss = _sceneRoot->getOrCreateStateSet();
    ss->setAttributeAndModes(lm);
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

    auto light0 = _viewer->getLight();
    light0->setDiffuse(sgc.getVec4("scene.headlight.diffuse"));
    light0->setSpecular(sgc.getVec4("scene.headlight.specular"));
    light0->setAmbient(sgc.getVec4("scene.headlight.ambient"));
    _viewer->setLightingMode(osg::View::HEADLIGHT);

    auto moonPos = sgc.getVec3("starfield.moon.pos");
    moonPos.normalize();
    auto ls = new osg::LightSource;
    auto light1 = ls->getLight();
    light1->setLightNum(1);
    light1->setPosition(osg::Vec4(moonPos, 0));
    light1->setAmbient(sgc.getVec4("starfield.moon.ambient"));
    light1->setDiffuse(sgc.getVec4("starfield.moon.diffuse"));
    light1->setSpecular(sgc.getVec4("starfield.moon.specular"));

    _sceneRoot->addChild(ls);
    ls->setStateSetModes(*ss, osg::StateAttribute::ON);

    if (_shadowMap)
    {
        _shadowMap->setLight(light1);
    }
}

void Game::setupCameraAndManipulator()
{
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
    manipulator->setGravity(sgc.getFloat("scene.gravity"));
    _viewer->home();
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

osgText::Text* createText(const std::string& name, const std::string& text,
    const osg::Vec4& color, int size, const osg::Vec3& pos)
{
    static auto font = osgText::readFontFile("font/font.ttf");

    auto t = new osgText::Text;
    t->setName(name);
    t->setDataVariance(osg::Object::DYNAMIC);
    t->setFont(font);
    t->setCharacterSize(size);
    t->setAxisAlignment(osgText::TextBase::XY_PLANE);
    t->setColor(color);
    t->setPosition(pos);
    t->setText(text);
    return t;
}

void Game::createUI()
{
    auto root = new osg::Group;
    root->setName("UIRoot");

    // create timer text, bar
    {
        _timerBar = osg::createTexturedQuadGeometry(
            osg::Vec3(), osg::Vec3(1, 0, 0), osg::Vec3(0, 1, 0));
        _timerBar->setName("TimerBar");

        auto ss = _timerBar->getOrCreateStateSet();

        ss->setAttributeAndModes(
            createProgram("shader/timer_bar.frag", osg::Shader::FRAGMENT));
        ss->addUniform(new osg::Uniform("size", osg::Vec2(1, 1)));
        auto percentUniform = new osg::Uniform("percent", 1.0f);
        ss->addUniform(percentUniform);
        _timerBar->setUpdateCallback(osgf::createCallback(
            [=](auto* obj, auto* data) { percentUniform->set(sgg.getPercentTime()); }));
    }

    _score = 0;
    auto fontSize = sgc.getInt("ui.font.size");
    _scoreText =
        createText("Score", "0", sgc.getColor("ui.score.color"), fontSize, osg::Vec3());
    _timerText =
        createText("Timer", "30", sgc.getColor("ui.timer.color"), fontSize, osg::Vec3());
    _msg = createText("Msg", "Press r to start new game.", sgc.getColor("ui.msg.color"),
        fontSize, osg::Vec3());

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
        setUseCursor(false);

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

    _uiRoot = root;
    _hudCamera->addChild(_uiRoot);
}

class MeteorSpawner : public osg::Callback
{
public:
    MeteorSpawner(const osg::Vec2& gaussRate) : _gaussRate(gaussRate) {}

    bool run(osg::Object* object, osg::Object* data) override
    {
        _time -= sgg.getDeltaTime();
        if (_time <= 0)
        {
            _time = clamp(gaussRand(_gaussRate), 0.001f, 2.0f);
            sgg.spawnMeteor();
        }
        return traverse(object, data);
    }

private:
    double _time = 0.2;
    osg::Vec2 _gaussRate;
};

void Game::createStarfield()
{
    // starfield is far far away, they don't conribute to depth buffer, they use
    // LEQUAL depth compare func. These drawables has a invalid BoundingBox, it's used to
    // supress near far calculation. It also need a Projection node since they don't
    // contribute to near far.
    // Moon and star is point sprite, meteor is rect, flying along it's -x

    auto root = new osg::Group;
    root->setNodeMask(nb_unreal_object);
    root->setName("Starfield");

    auto rootSS = root->getOrCreateStateSet();

    // render it between opaque and transparent, no depth write.
    rootSS->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 1.0f, 1.0f, false));
    rootSS->setAttributeAndModes(
        new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));

    auto radius = sgc.getFloat("starfield.radius");
    auto radius2 = radius * radius;

    auto projNode =
        new osg::Projection(createDefaultProjectionMatrix(radius * 0.5f, radius * 2.0f));
    root->addChild(projNode);

    // add moon
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
        ss->setAttributeAndModes(createProgram("shader/moon.vert", "shader/moon.frag"));
        ss->setRenderBinDetails(2, "RenderBin");

        projNode->addChild(moon);
    }

    // add stars
    {
        auto stars = new osg::Geometry;
        stars->setName("Star");
        auto vertices = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
        auto numStars = sgc.getInt("starfield.numStars");
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
        ss->setAttributeAndModes(createProgram("shader/star.vert", "shader/star.frag"));
        ss->setRenderBinDetails(2, "RenderBin");

        projNode->addChild(stars);
    }

    // add sky
    {
        auto sky = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), radius * 0.98));
        sky->setName("Sky");
        sky->setCullingActive(false);
        sky->setComputeBoundingBoxCallback(
            static_cast<osg::Drawable::ComputeBoundingBoxCallback*>(
                osgf::getInvalidComputeBoundingBoxCallback()));

        auto ss = sky->getOrCreateStateSet();

        ss->setMode(
            GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
        ss->setTextureAttributeAndModes(0, _noiseTexture3D);
        ss->setAttributeAndModes(createProgram("shader/sky.vert", "shader/sky.frag"));
        ss->setAttributeAndModes(new osg::BlendFunc(
            osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));

        auto moonDir = moonPos;
        moonDir.normalize();
        ss->addUniform(new osg::Uniform("moon", moonDir));
        ss->setRenderBinDetails(3, "RenderBin");

        projNode->addChild(sky);
    }

    _meteorStateSet = new osg::StateSet;
    _meteorStateSet->setAttributeAndModes(
        createProgram("shader/meteor.vert", "shader/meteor.frag"));
    _meteorStateSet->setRenderBinDetails(1, "RenderBin");
    root->addUpdateCallback(new MeteorSpawner(sgc.getVec2("starfield.meteor.rate.gauss")));

    _sceneRoot->addChild(root);
    _starfield = root;
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

    // render after grass, lightning don't write depth.
    ss->setRenderBinDetails(11, "DepthSortedBin");

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
    auto fontSize = sgc.getInt("ui.popscore.size");
    ;
    auto text = createText("PopScore", std::to_string(score),
        sgc.getColor("ui.popscore.color"), fontSize, pos);
    text->setNodeMask(nb_visible);
    text->setAlignment(osgText::Text::CENTER_CENTER);
    text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
    text->setColor(osg::Vec4(1, 0, 0, 1));
    text->setAutoRotateToScreen(true);

    auto ss = text->getOrCreateStateSet();
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    ss->setRenderBinDetails(11, "RenderBin");

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

    createStarfield();
}

class MeteorUpdater : public osg::Callback
{
public:
    MeteorUpdater(osg::MatrixTransform* meteor, const osg::Vec3& vel, const osg::Vec3& acc)
        : _meteor(meteor), _vel(vel), _acc(acc)
    {
    }

    bool run(osg::Object* object, osg::Object* data) override
    {
        auto starfield = object->asNode()->asGroup();
        assert(starfield);

        // remove meteor
        auto trans = _meteor->getMatrix().getTrans();
        if (trans.z() < -100)
        {
            assert(_meteor->getParent(0)->removeChild(_meteor));
            osg::ref_ptr<osg::Callback> cb = this;
            starfield->removeUpdateCallback(this);
            return cb->traverse(object, data);
        }

        auto dt = sgg.getDeltaTime();
        _vel += _acc * dt;
        _meteor->setMatrix(_meteor->getMatrix() * osg::Matrix::translate(_vel * dt));

        return traverse(object, data);
    }

private:
    osg::Vec3 _vel;
    osg::Vec3 _acc;
    osg::MatrixTransform* _meteor = 0;
};

void Game::spawnMeteor()
{
    // spawn outside of current view in view space
    assert(_starfield->getNumChildren() == 1);

    auto& projNode = dynamic_cast<osg::Projection&>(*_starfield->getChild(0));
    auto& projMatrix = projNode.getMatrix();
    double left, right, bottom, top, near, far;
    projMatrix.getFrustum(left, right, bottom, top, near, far);

    auto radius = sgc.getFloat("starfield.radius");
    auto width =
        clamp(gaussRand(sgc.getVec2("starfield.meteor.width.gauss")), 1.0f, 1000.0f);
    auto elevation =
        clamp(gaussRand(sgc.getVec2("starfield.meteor.elevation.gauss")), 0.0f, 80.0f);
    elevation = osg::DegreesToRadians(elevation);

    auto x = right * radius / near + width * 0.5f;
    auto y = radius * tan(elevation);
    auto pos = osg::Vec3(x, y, -radius);

    auto pitch = clamp(gaussRand(sgc.getVec2("starfield.meteor.pitch.gauss")), 0.0f, 90.0f);
    pitch = osg::DegreesToRadians(pitch);
    auto speed =
        clamp(gaussRand(sgc.getVec2("starfield.meteor.speed.gauss")), 1.0f, 1000.0f);
    auto vel = -osg::Vec3(cos(pitch), sin(pitch), 0) * speed;

    // clear pitch from view matrix, convert pos and dir to world space
    auto m = getMainCamera()->getInverseViewMatrix();
    auto forward = -getMatrixMajor3(m, 2);  // forward in world space
    forward.z() = 0;
    forward.normalize();
    auto up = osg::Z_AXIS;
    auto side = forward ^ up;
    side.normalize();
    setMatrixMajor3(m, 0, side);
    setMatrixMajor3(m, 1, up);
    setMatrixMajor3(m, 2, -forward);

    pos = pos * m;
    vel = osg::Matrix::transform3x3(vel, m);

    // create meteor
    auto height = width / sgc.getFloat("starfield.meteor.aspectRatio");
    auto meteor =
        osg::createTexturedQuadGeometry(osg::Vec3(-width * 0.5f, 0, -height * 0.5f),
            osg::Vec3(width, 0, 0), osg::Vec3(0, 0, height));
    meteor->setName("Meteor");
    meteor->setCullingActive(false);
    meteor->setComputeBoundingBoxCallback(
        static_cast<osg::Drawable::ComputeBoundingBoxCallback*>(
            osgf::getInvalidComputeBoundingBoxCallback()));
    meteor->setStateSet(_meteorStateSet);

    static auto count = 0;
    auto frame = new osg::MatrixTransform;
    auto ma = osg::Matrix::rotate(osg::X_AXIS, -vel);
    ma.postMultTranslate(pos);
    frame->setMatrix(ma);
    frame->addChild(meteor);
    frame->setName("Meteor" + std::to_string(count++));
    projNode.insertChild(projNode.getNumChildren() - 1, frame);

    auto acc = vel;
    acc.normalize();
    acc *=
        clamp(gaussRand(sgc.getVec2("starfield.meteor.acceleration.gauss")), 1.0f, 1000.0f);

    // animate it
    _starfield->addUpdateCallback(new MeteorUpdater(frame, vel, acc));
}

}  // namespace toy
