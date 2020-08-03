#include <GhostManipulator.h>

#include <Game.h>
#include <Config.h>

namespace toy
{

GhostManipulator::GhostManipulator()
{
    setAllowThrow(false);
}

bool GhostManipulator::handleFrame(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    auto walkDirection = combineWalkDirection();
    bool walking = walkDirection != osg::Vec3(0, 0, 0);
    bool retreat = walkDirection.y() < 0;

    auto cameraTransform = getMatrix();

    if (walking)
    {
        walkDirection.z() = 0;
        walkDirection.normalize();
        walkDirection *= retreat ? _walkSpeed * 0.5f : _walkSpeed;
        walkDirection *= sgg.getDeltaTime();

        auto forward = _rotation * -osg::Z_AXIS;
        auto side = _rotation * osg::X_AXIS;
        _eye += side * walkDirection.x() + forward * walkDirection.y();

        auto radius = sgg.getSceneRadius();
        _eye.x() = osg::clampBetween<float>(_eye.x(), -radius, radius);
        _eye.y() = osg::clampBetween<float>(_eye.y(), -radius, radius);

        auto tp = sgg.getTerrainPoint(_eye.x(), _eye.y());
        _eye.z() = tp.z() + _cameraHeight;
    }

    return FirstPersonManipulator::handleFrame(ea, us);
}

bool GhostManipulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
{
    switch (ea.getEventType())
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
            switch (ea.getUnmodifiedKey())
            {
                case osgGA::GUIEventAdapter::KEY_W:
                    addWalkDirection(wd_front);
                    break;
                case osgGA::GUIEventAdapter::KEY_S:
                    addWalkDirection(wd_back);
                    break;
                case osgGA::GUIEventAdapter::KEY_A:
                    addWalkDirection(wd_left);
                    break;
                case osgGA::GUIEventAdapter::KEY_D:
                    addWalkDirection(wd_right);
                    break;
                default:
                    break;
            }
            return false;
        case osgGA::GUIEventAdapter::KEYUP:
            switch (ea.getUnmodifiedKey())
            {
                case osgGA::GUIEventAdapter::KEY_W:
                    removeWalkDirection(wd_front);
                    break;
                case osgGA::GUIEventAdapter::KEY_S:
                    removeWalkDirection(wd_back);
                    break;
                case osgGA::GUIEventAdapter::KEY_A:
                    removeWalkDirection(wd_left);
                    break;
                case osgGA::GUIEventAdapter::KEY_D:
                    removeWalkDirection(wd_right);
                    break;
                default:
                    break;
            }

            return false;
        default:
            break;
    }

    return FirstPersonManipulator::handle(ea, us);
}

bool GhostManipulator::performMovementLeftMouseButton(
    const double eventTimeDelta, const double dx, const double dy)
{
    return false;
}

bool GhostManipulator::performMovementRightMouseButton(
    const double eventTimeDelta, const double dx, const double dy)
{
    return FirstPersonManipulator::performMovementLeftMouseButton(eventTimeDelta, dx, dy);
}

osg::Vec3 GhostManipulator::combineWalkDirection()
{
    osg::Vec3 d;
    if (_walkDirection & wd_left)
        d.x() += -1;
    if (_walkDirection & wd_right)
        d.x() += 1;
    if (_walkDirection & wd_front)
        d.y() += 1;
    if (_walkDirection & wd_back)
        d.y() += -1;
    return d;
}

}  // namespace toy
