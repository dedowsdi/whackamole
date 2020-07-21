#include <BaseGame.h>

#include <cassert>

#include <osg/Hint>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <ALBuffer.h>
#include <ALContext.h>
#include <ALListener.h>
#include <ALSource.h>
#include <OsgFactory.h>
#include <OsgQuery.h>
#include <Resource.h>

namespace toy
{

void BaseGame::clear() {}

void BaseGame::init(int argc, char* argv[], osgViewer::Viewer* viewer)
{
    _viewer = viewer;
    osg::DisplaySettings::instance()->setMinimumNumStencilBits(1);
    _viewer->realize();

    createRoots();
    _viewer->setSceneData(_root);
    createScene();

    createSound(argc, argv);
}

osg::Vec2i BaseGame::getWindowSize()
{
    auto rect = osgq::getWindowRect(*_viewer);
    return osg::Vec2i(rect.z(), rect.w());
}

osg::Vec2 BaseGame::getWindowCenter()
{
    auto size = getWindowSize();
    return osg::Vec2(size.x() * 0.5f, size.y() * 0.5f);
}

osg::Program* BaseGame::createProgram(const std::string& fragFile, int shaderType)
{
    auto prg = new osg::Program;

    auto fragShader =
        osgDB::readShaderFile(static_cast<osg::Shader::Type>(shaderType), fragFile);
    prg->addShader(fragShader);

#ifdef DEBUG
    _observer->addResource(createShaderResource(fragShader));
#endif /* ifndef DEBUG */

    return prg;
}

osg::Program* BaseGame::createProgram(const std::string& vertFile, const std::string& fragFile)
{
    auto prg = new osg::Program;

    auto vertShader = osgDB::readShaderFile(osg::Shader::VERTEX, vertFile);
    prg->addShader(vertShader);

    auto fragShader = osgDB::readShaderFile(osg::Shader::FRAGMENT, fragFile);
    prg->addShader(fragShader);

#ifdef DEBUG
    _observer->addResource(createShaderResource(vertShader));
    _observer->addResource(createShaderResource(fragShader));
#endif /* ifndef DEBUG */

    return prg;
}

osg::Program* BaseGame::createProgram(const std::string& vertFile, const std::string& geomFile,
    const std::string& fragFile, int inputType, int outputType, int maxVertices)
{
    auto prg = new osg::Program;
    prg->setParameter(GL_GEOMETRY_INPUT_TYPE_EXT, inputType);
    prg->setParameter(GL_GEOMETRY_OUTPUT_TYPE_EXT, outputType);
    prg->setParameter(GL_GEOMETRY_VERTICES_OUT_EXT, maxVertices);

    auto vertShader = osgDB::readShaderFile(osg::Shader::VERTEX, vertFile);
    prg->addShader(vertShader);

    auto geomShader = osgDB::readShaderFile(osg::Shader::GEOMETRY, geomFile);
    prg->addShader(geomShader);

    auto fragShader = osgDB::readShaderFile(osg::Shader::FRAGMENT, fragFile);
    prg->addShader(fragShader);

#ifdef DEBUG
    _observer->addResource(createShaderResource(vertShader));
    _observer->addResource(createShaderResource(fragShader));
    _observer->addResource(createShaderResource(geomShader));
#endif /* ifndef DEBUG */

    return prg;
}

void BaseGame::debugDrawLine(const osg::Vec3& from, const osg::Vec3& to,
    const osg::Vec4& fromColor, const osg::Vec4& toColor)
{
    if (!_debugLines)
    {
        _debugLines = new osg::Geometry;
        auto vertices = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
        auto colors = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX);
        _debugLines->setVertexArray(vertices);
        _debugLines->setColorArray(colors);
        _debugLines->setDataVariance(osg::Object::DYNAMIC);

        _debugLines->setName("DebugLines");
        _debugLines->setUseDisplayList(false);
        _debugLines->setUseVertexBufferObjects(true);
        _debugLines->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 0));

        _debugRoot->addChild(_debugLines);
    }

    auto vertices = static_cast<osg::Vec3Array*>(_debugLines->getVertexArray());
    auto colors = static_cast<osg::Vec4Array*>(_debugLines->getColorArray());
    vertices->push_back(from);
    vertices->push_back(to);
    colors->push_back(fromColor);
    colors->push_back(toColor);

    vertices->dirty();
    colors->dirty();

    static_cast<osg::DrawArrays*>(_debugLines->getPrimitiveSet(0))
        ->setCount(vertices->size());
    _debugLines->dirtyBound();
}

void BaseGame::debugDrawSphere(const osg::Vec3& pos, float radius, const osg::Vec4& color)
{
    _debugRoot->addChild(osgf::createSphereAt(pos, radius, color));
}

osg::Camera* BaseGame::getMainCamera() const
{
    return _viewer->getCamera();
}

void BaseGame::setALListener(ALListener* v)
{
    _alListener = v;
}

BaseGame::BaseGame() {}

BaseGame::~BaseGame() {}

void BaseGame::createSound(int argc, char* argv[])
{
    _alContext.reset(new ALContext(argc, argv));
    _alListener = new ALListener;

    // make sure listener is not updated before sources
    auto listenerUpdater = new ALListenerUpdater(_alListener);
    _sceneRoot2->addUpdateCallback(listenerUpdater);

    OSG_NOTICE << "OpenAL init finished.\n" << std::string(80, '*') << std::endl;
}

void BaseGame::createRoots()
{
    _root = new osg::Group();
    _root->setName("BaseGame");

#ifdef DEBUG
    _observer = new ResourceObserver;
    _root->addUpdateCallback(_observer);
#endif /* ifndef DEBUG */

    _debugRoot = new osg::Group();
    _debugRoot->setName("DebugRoot");

    _root->addChild(_debugRoot);

    _sceneRoot = new osg::Group();
    _sceneRoot->setName("SceneRoot");

    _root->addChild(_sceneRoot);

    _sceneRoot2 = new osg::Group();
    _sceneRoot2->setName("SceneRoot2");

    _root->addChild(_sceneRoot2);

    auto rect = osgq::getWindowRect(*_viewer);

    _hudCamera = osgf::createOrthoCamera(0, rect.z(), 0, rect.w());
    _hudCamera->setName("Hud");
    auto ss = _hudCamera->getOrCreateStateSet();
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    _root->addChild(_hudCamera);
}

}  // namespace toy
