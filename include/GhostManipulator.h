#ifndef WHACKAMOLE_GHOSTMANIPULATOR_H
#define WHACKAMOLE_GHOSTMANIPULATOR_H

#include <osgGA/FirstPersonManipulator>

namespace toy
{

class GhostManipulator : public osgGA::FirstPersonManipulator
{
public:
    GhostManipulator();

    float getWalkSpeed() const { return _walkSpeed; }
    void setWalkSpeed(float v) { _walkSpeed = v; }

protected:

    bool handleFrame(
        const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us) override;

    bool performMovementLeftMouseButton(
        const double eventTimeDelta, const double dx, const double dy) override;

    bool performMovementRightMouseButton(
        const double eventTimeDelta, const double dx, const double dy) override;

    enum walk_direction
    {
        wd_right = 1 << 0,
        wd_left = 1 << 1,
        wd_front = 1 << 2,
        wd_back = 1 << 3
    };

    void addWalkDirection(walk_direction d) { _walkDirection |= d; }

    void removeWalkDirection(walk_direction d) { _walkDirection &= ~d; }

    osg::Vec3 combineWalkDirection();

    int _walkDirection = 0;
    float _walkSpeed = 8.0f;
};

}  // namespace toy

#endif  // WHACKAMOLE_GHOSTMANIPULATOR_H
