#ifndef WHACKAMOLE_BASEGAME_H
#define WHACKAMOLE_BASEGAME_H

#include <memory>
#include <set>
#include <vector>
#include <string>

#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ref_ptr>

#define TOY_STR(x) TOY_STR2(x)
#define TOY_STR2(x) #x
#define TOY_HERE (__FILE__ ":" TOY_STR(__LINE__) ":")

namespace osg
{
class Group;
class Vec2i;
class Texture2D;
class Camera;
class Geometry;
class Program;
class Shader;
}  // namespace osg

namespace osgGA
{
class CameraManipulator;
}

namespace osgViewer
{
class Viewer;
};

namespace toy
{

class ALBuffer;
class ALSource;
class ALContext;
class ALListener;
class ResourceObserver;

class BaseGame
{
public:
    BaseGame();

    virtual ~BaseGame();

    void clear();

    void init(int argc, char* argv[], osgViewer::Viewer* viewer);

    // Root children is fixed, don't add or remove anything to it during gaming.
    // If you want to manipulate children of _sceneRoot, make sure it happens in
    // update or event traversal of root.
    osg::Group* getRoot() { return _root; }

    osg::Group* getDebugRoot() { return _debugRoot; }

    osg::Vec2i getWindowSize();

    osg::Vec2 getWindowCenter();

    // Create program and update it if ctime of shader file changed
    osg::Program* createProgram(const std::string& fragFile, int shaderType);

    osg::Program* createProgram(const std::string& vertFile, const std::string& fragFile);

    osg::Program* createProgram(const std::string& vertFile, const std::string& geomFile,
        const std::string& fragFile, int inputType, int outputType, int maxVertices);

    void setUseCursor(bool b);

    void debugDrawLine(const osg::Vec3& from, const osg::Vec3& to,
        const osg::Vec4& fromColor, const osg::Vec4& toColor);

    void debugDrawSphere(const osg::Vec3& pos, float radius, const osg::Vec4& color);

    osg::Camera* getMainCamera() const;

    osgViewer::Viewer* getViewer() { return _viewer; }
    const osgViewer::Viewer* getViewer() const { return _viewer; }
    void setViewer(osgViewer::Viewer* v) { _viewer = v; }

    osg::Group* getSceneRoot() const { return _sceneRoot; }
    void setSceneRoot(osg::Group* v) { _sceneRoot = v; }

    osg::Group* getSceneRoot2() const { return _sceneRoot2; }
    void setSceneRoot2(osg::Group* v) { _sceneRoot2 = v; }

    ALListener* getEar() const { return _alListener; }
    ALListener* getALListener() const { return _alListener; }
    void setALListener(ALListener* v);

    osg::Camera* getHudCamera() const { return _hudCamera; }
    void setHudCamera(osg::Camera* v) { _hudCamera = v; }

protected:
    virtual void createScene(){}

    virtual void preInit(){}

    virtual void postInit(){}

    void createSound(int argc, char* argv[]);

    void createRoots();

    std::unique_ptr<ALContext> _alContext;

    osgViewer::Viewer* _viewer = 0;

    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osg::Group> _sceneRoot;
    osg::ref_ptr<osg::Group> _sceneRoot2;
    osg::ref_ptr<osg::Group> _debugRoot;
    osg::ref_ptr<osg::Camera> _hudCamera;

    osg::ref_ptr<osg::Geometry> _debugLines;

    osg::ref_ptr<ALListener> _alListener;

    osg::ref_ptr<ResourceObserver> _observer;
};

}  // namespace toy

#endif  // WHACKAMOLE_BASEGAME_H
