#include <Math.h>

#include <osg/DisplaySettings>

namespace toy
{

osg::Matrix createDefaultProjectionMatrix(float near, float far)
{
    double height = osg::DisplaySettings::instance()->getScreenHeight();
    double width = osg::DisplaySettings::instance()->getScreenWidth();
    double distance = osg::DisplaySettings::instance()->getScreenDistance();
    double vfov = osg::RadiansToDegrees(atan2(height / 2.0f, distance) * 2.0);

    return osg::Matrixf::perspective(vfov, width / height, near, far);
}

namespace
{

inline void setForwardUpMajor(osg::Matrix& m, const osg::Vec3& v, Axis x)
{
    auto index = static_cast<int>(x) / 2;
    if (static_cast<int>(x) % 2 == 0)
    {
        setMatrixMajor3(m, index, v);
    }
    else
    {
        setMatrixMajor3(m, index, v * -1);
    }
}

}  // namespace

osg::Matrix forwardUp(const osg::Vec3& forward, const osg::Vec3& up, Axis forwardAxis,
    Axis upAxis, Axis sideAxis)
{
    auto f = forward;
    f.normalize();

    auto s = f ^ up;
    s.normalize();

    auto u = s ^ f;
    // u.normalize();

    osg::Matrix m;

    setForwardUpMajor(m, s, sideAxis);
    setForwardUpMajor(m, u, upAxis);
    setForwardUpMajor(m, f, forwardAxis);

    return m;
}

}  // namespace toy
