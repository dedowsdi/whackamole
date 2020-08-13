#ifndef WHACKAMOLE_TOYSHADOWMAP_H
#define WHACKAMOLE_TOYSHADOWMAP_H

#include <osgShadow/ShadowMap>

namespace toy
{

// osgShadow::ShadowMap render shadow depth for the entire scene, it caluse precision
// problem for large scene. This shadow map use fixed projection around user, it create
// shadow map only when current camera is _mainCamera.
class ToyShadowMap : public osgShadow::ShadowMap
{
public:
    ToyShadowMap() = default;
    ~ToyShadowMap() = default;

    osg::Camera* getMainCamera() const { return _mainCamera; }
    void setMainCamera(osg::Camera* v) { _mainCamera = v; }

    osg::Program* getProgram() { return _program.get(); }

    void cull(osgUtil::CullVisitor& cv) override;

    void init() override;

    const osg::Vec2& getProjectionSize() const { return _projectionSize; }
    void setProjectionSize(const osg::Vec2& v) { _projectionSize = v; }

    const osg::Vec3& getCenter() const { return _center; }
    void setCenter(const osg::Vec3& v) { _center = v; }

private:
    osg::Vec2 _projectionSize;
    osg::Vec3 _center;

    osg::Uniform* _shadowMatrix = 0;
    osg::Camera* _mainCamera = 0;
};

}  // namespace toy

#endif  // WHACKAMOLE_TOYSHADOWMAP_H
