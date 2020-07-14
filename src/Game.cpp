#include <Game.h>

#include <cassert>
#include <iomanip>

#include <osg/AlphaFunc>
#include <osg/AnimationPath>
#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/Layer>
#include <osgTerrain/Locator>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgText/Text>
#include <osgUtil/PerlinNoise>
#include <osgViewer/Viewer>
#include <Lightning.h>
#include <Math.h>
#include <OsgFactory.h>

namespace toy
{

const int burrows = 8;
const float sceneRadius = 128;
const float sceneHeight = 32;
const int terrainCols = 65;
const int terrainRows = 65;
const float lawnHeight = 2.0f;
const float burrowRadius = 6.0f;
const float burrowHeight = 6.0f;
// const int numGrass = 128;
const float grassSize = 8;

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
    auto t0 = visitor->getFrameStamp()->getSimulationTime();
    _deltaTime = _lastTime == 0 ? 0 : t0 - _lastTime;
    _lastTime = t0;

    if (_status == gs_running)
    {
        auto newMole = toy::unitRand() > 0.99;
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
    startPos -= burrow.normal * Mole::getDrawableBoundingBox().zMax();
    auto endPos = burrow.pos;
    endPos -= burrow.normal * Mole::getDrawableBoundingBox().zMin();

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
    playKickAnimation(mole->getMatrix().getTrans());

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
    OSG_NOTICE << "Remove mole " << mole->getName() << std::endl;
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
    _sceneRoot->addChild(createTerrain());
    _sceneRoot->addChild(createMeadow());

    _burrowList.clear();
    createBurrows();

    _msg->setNodeMask(0);
    _timer = 30;
    _totalTime = 30;
    _timerText->setText(std::to_string(_timer));

    show(_scoreText);
    show(_timerText);
    show(_timerBar);

    _status = gs_running;

    _viewer->getCameraManipulator()->home(0);
}

void Game::timeout()
{
    show(_msg);
    _msg->setText("Press r to start new game.");
    _status = gs_timeout;
}

void Game::hide(osg::Node* node)
{
    node->setNodeMask(0);
}

void Game::show(osg::Node* node)
{
    node->setNodeMask(1);
}

osg::Node* Game::createTerrain()
{
    _heightField = new osg::HeightField;

    auto rows = terrainRows;
    auto cols = terrainCols;
    auto xInterval = 1;
    auto yInterval = 1;

    _heightField->allocate(rows, cols);
    _heightField->setXInterval(xInterval);
    _heightField->setYInterval(yInterval);
    _heightField->setOrigin(osg::Vec3());

    osgUtil::PerlinNoise pn;
    pn.SetNoiseFrequency(64);

    auto yStep = 0.03;
    auto xStep = 0.03;
    double v[2] = {0, 0};

    for (int y = 0; y < rows; ++y)
    {
        v[1] += yStep;
        v[0] = 0;
        for (int x = 0; x < cols; ++x)
        {
            v[0] += xStep;
            auto h = pn.PerlinNoise2D(v[0], v[1], 2, 2, 3) * sceneHeight;
            _heightField->setHeight(x, y, h);
        }
    }

    auto locator = new osgTerrain::Locator;
    locator->setCoordinateSystemType(osgTerrain::Locator::GEOGRAPHIC);
    locator->setTransformAsExtents(-sceneRadius, -sceneRadius, sceneRadius, sceneRadius);

    auto layer = new osgTerrain::HeightFieldLayer(_heightField);
    layer->setLocator(locator);

    auto clayer = new osgTerrain::ImageLayer(osgDB::readImageFile("texture/ground.jpg"));

    auto tile = new osgTerrain::TerrainTile;
    tile->setTerrainTechnique(new osgTerrain::GeometryTechnique);
    tile->setTileID(osgTerrain::TileID(0, 0, 0));
    tile->setElevationLayer(layer);
    tile->setColorLayer(0, clayer);

    _terrain = new osgTerrain::Terrain;
    _terrain->addChild(tile);

    auto ss = _terrain->getOrCreateStateSet();

    auto material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0.5, 0.5, 0.5));
    ss->setAttributeAndModes(material);

    // return new osg::ShapeDrawable(_heightField);
    return _terrain;
}

osg::Geometry* createGrass()
{
    auto geometry = new osg::Geometry;
    geometry->setName("Grass");

    auto vertices = new osg::Vec3Array();
    auto texcoords = new osg::Vec2Array();
    // vertices->reserve( * 6 * 4);

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

osg::Node* Game::createMeadow()
{
    auto root = new osg::Group;
    auto pos = osg::Vec2();

    auto grass = createGrass();
    auto origin = osg::Vec2(-sceneRadius, -sceneRadius);

    auto stepSize = grassSize * 0.65f;

    auto cols = sceneRadius * 2 / stepSize;
    auto rows = sceneRadius * 2 / stepSize;

    for (auto i = 1; i < cols - 1; ++i)
    {
        pos.y() = i % 2 ? 0 : 0.5 * stepSize;
        pos.x() += stepSize;

        for (auto j = 1; j < rows - 1; ++j)
        {
            pos.y() += stepSize;

            auto p = osg::Vec2(origin + pos + toy::diskRand(stepSize * 0.25));
            auto tp = getTerrainPoint(p.x(), p.y());
            auto m = osg::Matrix::rotate(unitRand() * osg::PI_2f, osg::Z_AXIS);
            m.postMultTranslate(tp.first);

            auto frame = new osg::MatrixTransform;
            frame->setMatrix(m);
            frame->addChild(grass);
            root->addChild(frame);
        }
    }

    osg::StateSet* ss = root->getOrCreateStateSet();

    auto texture = new osg::Texture2D(osgDB::readImageFile("texture/grass0.png"));
    texture->setResizeNonPowerOfTwoHint(false);
    ss->setTextureAttributeAndModes(0, texture);

    auto material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8, 0.8, 0.8, 0.8));

    ss->setAttributeAndModes(material);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    ss->setAttributeAndModes(
        new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA));

    return root;
}

osg::Node* Game::createOverallMeadow()
{
    auto geometry = new osg::Geometry;
    auto vertices = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    auto texcoords = new osg::Vec2Array(osg::Array::BIND_PER_VERTEX);
    auto normals = new osg::Vec3Array(osg::Array::BIND_OVERALL);
    normals->push_back(osg::Z_AXIS);

    auto points = poissonDiskSample(
        osg::Vec2(-sceneRadius, -sceneRadius), osg::Vec2(sceneRadius, sceneRadius), 16, 32);

    vertices->reserve(points.size() * 6 * 4);
    texcoords->reserve(vertices->capacity());

    auto hsize = grassSize * 0.5;

    for (auto& p: points)
    {
        auto tp = getTerrainPoint(p.x(), p.y());
        auto& pos = tp.first;
        auto& normal = tp.second;

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

    return geometry;
}

std::pair<osg::Vec3, osg::Vec3> Game::getTerrainPoint(float x, float y)
{
    auto minX = -sceneRadius;
    auto minY = -sceneRadius;
    auto xInterval = sceneRadius * 2.0 / (terrainCols - 1);
    auto yInterval = sceneRadius * 2.0 / (terrainRows - 1);

    auto col = (x - minX) / xInterval;
    auto row = (y - minY) / yInterval;
    return std::make_pair(osg::Vec3(x, y, _heightField->getHeight(col, row)),
        _heightField->getNormal(col, row));
}

void Game::createBurrows()
{
    auto maxCenter = sceneRadius - burrowRadius;
    auto points = poissonDiskSample(
        osg::Vec2(-sceneRadius, -sceneRadius), osg::Vec2(sceneRadius, sceneRadius), 50, 50);

    for (auto p: points)
    {
        auto tp = getTerrainPoint(p.x(), p.y());
        auto burrow = createBurrow(tp.first, tp.second);
        _sceneRoot->addChild(burrow.node);
        _burrowList.push_back(burrow);
    }
}

Burrow Game::createBurrow(const osg::Vec3& pos, const osg::Vec3& normal)
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

    auto p = pos + normal * 2;
    auto frame = new osg::MatrixTransform;
    osg::Matrix m = osg::Matrix::rotate(osg::Z_AXIS, normal);
    m.postMultTranslate(p);
    frame->setMatrix(m);
    frame->addChild(graph);
    return Burrow{false, -1, p, normal, frame};
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
    auto wsize = getWindowSize();
    auto barSize = osg::Vec2(wsize.x() * 0.95, wsize.y() * 0.04);
    auto y = 16;

    // create timer text, bar
    {
        // auto barSize = osg::Vec2(bbTimer.xMin() - 4 - bbScore.xMax(), 50);
        _timerBar = osg::createTexturedQuadGeometry(osg::Vec3(wsize.x() * 0.025, y, 0),
            osg::Vec3(barSize.x(), 0, 0), osg::Vec3(0, barSize.y(), 0));
        y += barSize.y() + 2;

        auto ss = _timerBar->getOrCreateStateSet();

        auto prg = createProgram("shader/timer_bar.frag");
        ss->setAttributeAndModes(prg);

        ss->addUniform(new osg::Uniform("size", barSize));
        auto percentUniform = new osg::Uniform("percent", 1.0f);
        ss->addUniform(percentUniform);
        ss->setUpdateCallback(new TimerBarUpdater(percentUniform));
    }

    _score = 0;
    _scoreText = createText("Score", "0", 18, osg::Vec3(10, y, 0));
    _timerText = createText("Timer", "30", 18, osg::Vec3(wsize.x() * 0.975f, y, 0));
    _timerText->setAlignment(osgText::Text::RIGHT_BOTTOM);

    _msg = createText(
        "Msg", "Press r to start new game.", 18, osg::Vec3(20, wsize.y() - 20, 0));
    _msg->setAlignment(osgText::Text::LEFT_TOP);

    auto root = new osg::Group;
    root->addChild(_scoreText);
    root->addChild(_msg);
    root->addChild(_timerText);
    root->addChild(_timerBar);

    hide(_scoreText);
    hide(_timerText);
    hide(_timerBar);

    _timerText->setNodeMask(0);

    auto ss = root->getOrCreateStateSet();

    return root;
}

void Game::playKickAnimation(const osg::Vec3& pos)
{
    auto l = new Lightning;
    l->setBillboard(true);
    l->setBillboardWidth(5);
    l->setMaxJitter(0.17);
    l->setMaxForkAngle(osg::PIf * 0.025f);
    l->setBillboardType(Lightning::bt_per_line_local);
    l->add(8, pos + osg::Vec3(diskRand(5), 200), pos);

    _sceneRoot->addChild(l);
    _sceneRoot->addUpdateCallback(osgf::createTimerRemoveNodeUpdateCallback(1, l));
}

Game::Game() {}

Game::~Game() {}

}  // namespace toy
