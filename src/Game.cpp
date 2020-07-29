#include <Game.h>

#include <cassert>
#include <iomanip>

#include <osg/AlphaFunc>
#include <osg/AnimationPath>
#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geometry>
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
#include <ToyMath.h>

namespace toy
{

#define TOY_INFO OSG_INFO

osg::Vec3 Burrow::getTopCenter()
{
    return osg::Vec3(0, 0, sgc.getFloat("burrow.height") * 0.5f) * node->getMatrix();
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

        ss->setMode(
            GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);

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

    auto ksm = new osgGA::KeySwitchMatrixManipulator;
    ksm->addMatrixManipulator(
        osgGA::GUIEventAdapter::KEY_F7, "Trackball", new osgGA::TrackballManipulator);
    ksm->addMatrixManipulator(
        osgGA::GUIEventAdapter::KEY_F8, "Ghost", new GhostManipulator);

    _viewer->setCameraManipulator(ksm);
}

void Game::postInit()
{
    _viewer->home();
}

void Game::createScene()
{
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
    showReal(mole);
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

    show(mole);
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
    sgc.reload();
    _sceneRadius = sgc.getFloat("scene.radius");
    _sceneHeight = sgc.getFloat("scene.height");
    _popRate = sgc.getFloat("mole.popRate");

    _score = 0;
    _scoreText->setText("0");

    _sceneRoot->removeChild(0, _sceneRoot->getNumChildren());
    _sceneRoot->setUpdateCallback(0);
    _removeMoleCallbacks.clear();

    auto camera = getMainCamera();
    camera->setClearColor(osg::Vec4(0.1, 0.1, 0.1, 0.1));

    createStarfield();
    createTerrain();
    createMeadow();

    _burrowList.clear();
    createBurrows();
    _explosions.assign(16, osg::Vec4());

    hide(_msg);
    _timer = 60;
    _totalTime = 60;
    _timerText->setText(std::to_string(_timer));

    show(_scoreText);
    show(_timerText);
    show(_timerBar);

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
}

void Game::timeout()
{
    show(_msg);
    _msg->setText("Press r to start new game.");
    _status = gs_timeout;
}

void Game::resize(int width, int height)
{
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
    (*vertices)[0] = corner+heightVec;
    (*vertices)[1] = corner;
    (*vertices)[2] = corner+widthVec;
    (*vertices)[3] = corner+widthVec+heightVec;
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

void Game::show(osg::Node* node)
{
    node->setNodeMask(nb_visible);
}

void Game::showReal(osg::Node* node)
{
    node->setNodeMask(nb_visible | nb_raytest);
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
    // HeightField::getNormal will broke
    auto rows = sgc.getInt("terrain.rows");
    auto cols = sgc.getInt("terrain.cols");
    auto xInterval = _sceneRadius * 2 / cols;
    auto yInterval = _sceneRadius * 2 / rows;

    _heightField->allocate(rows, cols);
    _heightField->setXInterval(xInterval);
    _heightField->setYInterval(yInterval);
    _heightField->setOrigin(osg::Vec3());

    osgUtil::PerlinNoise pn;
    pn.SetNoiseFrequency(64);

    auto yStep = 0.03;
    auto xStep = 0.03;
    double v[2] = {linearRand(-100.0f, 100.0f), linearRand(-100.0f, 100.0f)};

    for (int y = 0; y < rows; ++y)
    {
        v[1] += yStep;
        v[0] = 0;
        for (int x = 0; x < cols; ++x)
        {
            v[0] += xStep;
            auto h = pn.PerlinNoise2D(v[0], v[1], 2, 2, 3) * _sceneHeight;
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
    tile->setTerrain(_terrain);
    _terrain->addChild(tile);

    auto ss = _terrain->getOrCreateStateSet();

    auto material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0.5, 0.5, 0.5));
    ss->setAttributeAndModes(material);

    _sceneRoot->addChild(_terrain);
}

osg::Geometry* createGrass()
{
    auto geometry = new osg::Geometry;
    geometry->setName("Grass");

    auto vertices = new osg::Vec3Array();
    auto texcoords = new osg::Vec2Array();
    // vertices->reserve( * 6 * 4);

    auto grassSize = sgc.getFloat("meadow.grass.size");
    auto hsize = grassSize * 0.5;
    for (auto j = 0; j < 3; ++j)
    {
        auto angle = osg::PI * j / 3;

        auto p0 = osg::Vec3(-cos(angle) * hsize, sin(angle) * hsize, 0);
        auto p1 = osg::Vec3(-p0.x(), -p0.y(), 0);
        auto p2 = osg::Vec3(p1.x(), p1.y(), grassSize);
        auto p3 = osg::Vec3(p0.x(), p0.y(), grassSize);

        vertices->push_back(p0);
        vertices->push_back(p1);
        vertices->push_back(p2);

        vertices->push_back(p0);
        vertices->push_back(p2);
        vertices->push_back(p3);

        texcoords->push_back(osg::Vec2(0, 0));
        texcoords->push_back(osg::Vec2(1, 0));
        texcoords->push_back(osg::Vec2(1, 1));

        texcoords->push_back(osg::Vec2(0, 0));
        texcoords->push_back(osg::Vec2(1, 1));
        texcoords->push_back(osg::Vec2(0, 1));
    }

    geometry->setVertexArray(vertices);
    geometry->setTexCoordArray(0, texcoords);
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertices->size()));

    return geometry;
}

void Game::createMeadow()
{
    auto root = new osg::Group;
    show(root);

    auto pos = osg::Vec2();

    auto grass = createGrass();
    auto origin = osg::Vec2(-_sceneRadius, -_sceneRadius);

    auto step = sgc.getFloat("meadow.grass.step") * sgc.getFloat("meadow.grass.size");

    auto numGroups = sgc.getInt("meadow.numGroupsPerRow");

    // create grass groups to speed up cull traversal
    std::map<std::string, osg::Group*> grassGroups;
    for (auto i = 0; i < numGroups; i++)
    {
        for (auto j = 0; j < numGroups; j++)
        {
            auto group = new osg::Group;
            group->setName("GrassGroup" + std::to_string(i) + ":" + std::to_string(j));
            root->addChild(group);
            grassGroups[group->getName()] = group;
        }
    }

    // create grasses
    int cols = _sceneRadius * 2 / step;
    int rows = _sceneRadius * 2 / step;
    for (auto i = 1; i < cols - 1; ++i)
    {
        pos.y() = i % 2 ? 0 : 0.5 * step;
        pos.x() += step;

        for (auto j = 1; j < rows - 1; ++j)
        {
            pos.y() += step;

            auto p = osg::Vec2(origin + pos + toy::diskRand(step * 0.25));
            auto tp = getTerrainPoint(p.x(), p.y());
            auto m = osg::Matrix::rotate(unitRand() * osg::PI_2f, osg::Z_AXIS);
            m.postMultTranslate(tp);

            auto frame = new osg::MatrixTransform;
            frame->setMatrix(m);
            frame->addChild(grass);

            auto groupCol = numGroups * i / cols ;
            auto groupRow = numGroups * j / rows ;
            auto groupName =
                "GrassGroup" + std::to_string(groupCol) + ":" + std::to_string(groupRow);
            auto parentGroup = grassGroups[groupName];

            parentGroup->addChild(frame);
            // OSG_DEBUG << "add grass " << i << ":" << j << " to group " << groupName
            //            << std::endl;
        }
    }

    static osg::StateSet* ss = 0;
    if (!ss)
    {
        ss = root->getOrCreateStateSet();

        auto texture = new osg::Texture2D(osgDB::readImageFile("texture/grass0.png"));
        texture->setResizeNonPowerOfTwoHint(false);
        ss->setTextureAttributeAndModes(0, texture);

        auto material = new osg::Material;
        material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8, 0.8, 0.8, 0.8));

        ss->setAttributeAndModes(material);
        ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        ss->setAttributeAndModes(new osg::BlendFunc(
            osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));

        auto prg = createProgram("shader/grass.vert", "shader/grass.frag");
        ss->setAttributeAndModes(prg);

        ss->addUniform(new osg::Uniform("viewMatrix", osg::Matrixf()));
        ss->addUniform(new osg::Uniform(osg::Uniform::FLOAT_VEC4, "explosions", 16));
    }
    root->setStateSet(ss);
    auto uf = ss->getUniform("explosions");

    // update explosion uniform
    root->addCullCallback(osgf::createCallback([=](osg::Object* obj, osg::Object* data) {
        auto& viewMatrix =
            data->asNodeVisitor()->asCullVisitor()->getCurrentCamera()->getViewMatrix();
        for (auto i = 0; i < _explosions.size(); ++i)
        {
            auto& e = _explosions[i];
            osg::Vec4 v = osg::Vec4(e.x(), e.y(), e.z(), 1) * viewMatrix;
            v.w() = e.w();
            uf->setElement(i, v);
        }
    }));

    _sceneRoot->addChild(root);
}

void Game::createOverallMeadow()
{
    auto geometry = new osg::Geometry;
    auto vertices = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    auto texcoords = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX);
    auto normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
    normals->push_back(osg::Z_AXIS);

    auto points = poissonDiskSample(osg::Vec2(-_sceneRadius, -_sceneRadius),
        osg::Vec2(_sceneRadius, _sceneRadius), 16, 32);

    vertices->reserve(points.size() * 6 * 4);
    texcoords->reserve(vertices->capacity());

    auto grassSize = sgc.getFloat("meadow.grass.size");
    auto offsetValue = sgc.getFloat("meadow.grass.maxOffset");
    auto maxOffset = osg::Vec2(offsetValue, offsetValue);
    auto hsize = grassSize * 0.5;

    for (auto& p: points)
    {
        auto xy = p + linearRand(-maxOffset, maxOffset) * grassSize;
        auto pos = getTerrainPoint(xy.x(), xy.y());
        auto startAngle = unitRand() * osg::PI_2f;

        // create * billboards
        for (auto j = 0; j < 3; ++j)
        {
            auto angle = startAngle + osg::PI * j / 3;

            auto p0 = pos + osg::Vec3(-cos(angle) * hsize, -sin(angle) * hsize, 0);
            auto p1 = pos + osg::Vec3(cos(angle) * hsize, sin(angle) * hsize, 0);
            auto p2 = p1 + osg::Z_AXIS * grassSize;
            auto p3 = p0 + osg::Z_AXIS * grassSize;

            vertices->push_back(p0);
            vertices->push_back(p1);
            vertices->push_back(p2);

            vertices->push_back(p0);
            vertices->push_back(p2);
            vertices->push_back(p3);

            texcoords->push_back(osg::Vec2(0, 0));
            texcoords->push_back(osg::Vec2(1, 0));
            texcoords->push_back(osg::Vec2(1, 1));

            texcoords->push_back(osg::Vec2(0, 0));
            texcoords->push_back(osg::Vec2(1, 1));
            texcoords->push_back(osg::Vec2(0, 1));
        }
    }

    geometry->setVertexArray(vertices);
    geometry->setTexCoordArray(0, texcoords);
    geometry->setNormalArray(normals);
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertices->size()));

    auto texture = new osg::Texture2D(osgDB::readImageFile("texture/grass0.png"));
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);

    osg::StateSet* ss = geometry->getOrCreateStateSet();
    ss->setTextureAttributeAndModes(0, texture);
    ss->setAttributeAndModes(new osg::AlphaFunc(osg::AlphaFunc::GREATER, 0));
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    // ss->setMode(GL_BLEND, osg::StateAttribute::ON);

    _sceneRoot->addChild(geometry);
}

void Game::createBurrows()
{
    auto maxCenter = _sceneRadius - sgc.getFloat("burrow.radius");
    auto spawnRadius = sgc.getFloat("burrow.spawnRadius") * _sceneRadius;
    auto spawnDistance = sgc.getFloat("burrow.spawnDistance");
    auto points = poissonDiskSample(osg::Vec2(-spawnRadius, -spawnRadius),
        osg::Vec2(spawnRadius, spawnRadius), spawnDistance, 32);

    for (auto& p: points)
    {
        auto point = getTerrainPoint(p.x(), p.y());
        auto normal = getTerrainNormal(p.x(), p.y());
        normal.normalize();
        auto burrow = createBurrow(point, normal);
        _sceneRoot->addChild(burrow.node);
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
    showReal(frame);
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
    _msg = createText(
        "Msg", "Press r to start new game.", 18, osg::Vec3());

    auto root = new osg::Group;
    root->addChild(_scoreText);
    root->addChild(_msg);
    root->addChild(_timerText);
    root->addChild(_timerBar);
    show(root);

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

    auto wsize = osgq::getWindowRect(*_viewer);
    resize(wsize.z(), wsize.w());

    return root;
}

void Game::createStarfield()
{
    auto root = new osg::Group;
    show(root);
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

        auto point = new osg::Point(128);
        point->setMaxSize(1024);

        auto ss = moon->getOrCreateStateSet();
        ss->setAttributeAndModes(point);
        ss->setTextureAttributeAndModes(0, new osg::PointSprite());
        ss->setAttributeAndModes(new osg::BlendFunc(
            osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));

        static auto program = createProgram("shader/moon.frag", osg::Shader::FRAGMENT);
        rootSS->setAttributeAndModes(program);

        projNode->addChild(moon);
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
        ss->setAttributeAndModes(new osg::Point(12));
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
    show(l);
    l->setBillboardWidth(8);
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
    }

    *iter = osg::Vec4(pos, sgc.getFloat("lightning.explosionForce"));
    auto index = std::distance(_explosions.begin(), iter);
    auto explosionUpdater = osgf::createCallback(
        [=](auto obj, auto data) { _explosions[index].w() -= sgg.getDeltaTime() * 5; });
    _sceneRoot->addUpdateCallback(explosionUpdater);
    _sceneRoot->addUpdateCallback(osgf::createTimerUpdateCallback(2.5,
        [=](auto obj, auto data) { _sceneRoot->removeUpdateCallback(explosionUpdater); }));
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
