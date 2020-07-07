#include <Game.h>

#include <cassert>

#include <osg/AnimationPath>
#include <osg/Billboard>
#include <osg/BlendFunc>
#include <osg/ClipPlane>
#include <osg/Hint>
#include <osg/LightModel>
#include <osg/LineWidth>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgViewer/Viewer>

#include <ALBuffer.h>
#include <ALContext.h>
#include <ALListener.h>
#include <ALSource.h>
#include <Math.h>
#include <OsgFactory.h>
#include <OsgQuery.h>

namespace toy
{

void Game::clear() {}

void Game::init(int argc, char* argv[], osgViewer::Viewer* viewer)
{
    _viewer = viewer;
    _viewer->realize();

    createScene();
    _viewer->setSceneData(_root);

    createSound(argc, argv);
}

osg::Vec2i Game::getWindowSize()
{
    auto rect = osgq::getWindowRect(*_viewer);
    return osg::Vec2i(rect.z(), rect.w());
}

osg::Vec2 Game::getWindowCenter()
{
    auto size = getWindowSize();
    return osg::Vec2(size.x() * 0.5f, size.y() * 0.5f);
}

void Game::debugDrawLine(const osg::Vec3& from, const osg::Vec3& to,
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

        _debugLines->setName("Game#DebugLines");
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

void Game::debugDrawSphere(const osg::Vec3& pos, float radius, const osg::Vec4& color)
{
    _debugRoot->addChild(osgf::createSphereAt(pos, radius, color));
}

osg::Camera* Game::getMainCamera() const
{
    return _viewer->getCamera();
}

void Game::setALListener(ALListener* v)
{
    _alListener = v;
}

Game::Game() {}

Game::~Game() {}

void Game::createScene()
{
    createRoots();
}

void Game::createSound(int argc, char* argv[])
{
    _alContext.reset(new ALContext(argc, argv));
    _alListener = new ALListener;

    // make sure listener is not updated before sources
    auto listenerUpdater = new ALListenerUpdater(_alListener);
    _sceneRoot2->addUpdateCallback(listenerUpdater);

    OSG_NOTICE << "OpenAL init finished.\n" << std::string(80, '*') << std::endl;
}

void Game::createRoots()
{
    _root = new osg::Group();
    _root->setName("Game");

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

    _hudCamera = osgf::createOrthoCamera(rect.x(), rect.y(), rect.z(), rect.w());
    _root->addChild(_hudCamera);
}

}  // namespace toy
