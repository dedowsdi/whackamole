#ifndef BOUNCINGBALL_ALLISTENER_H
#define BOUNCINGBALL_ALLISTENER_H

#include <osg/Node>

namespace toy
{
class ALListener;
class ALListenerUpdater : public osg::Callback
{
public:
    ALListenerUpdater(ALListener* listener) : _listener(listener) {}

    bool run(osg::Object* node, osg::Object* data) override;

private:
    ALListener* _listener = 0;
};

// Use world position as position.
// Us world rotation as orientation.
// It's applied in update traversal.
class ALListener : public osg::Node
{
public:
    ALListener() = default;

    const char* className() const override { return "ALListener"; }

    const char* libraryName() const override { return "toy"; }

    float getGain() const { return _gain; }
    void setGain(float v);

    osg::Vec3 getVelocity() const { return _velocity; }
    void setVelocity(osg::Vec3 v);

    void apply();
    void applyAttributes();

private:
    void dirty() { _dirty = true; }

    bool _dirty = false;  // only used for gain
    float _gain = 1.0f;
    osg::Vec3f _velocity;
};

}  // namespace toy
#endif  // BOUNCINGBALL_ALLISTENER_H
