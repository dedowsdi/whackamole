#include <ALListener.h>

#include <osg/NodeVisitor>

#include <AL.h>

namespace toy
{
bool ALListenerUpdater::run(osg::Object* node, osg::Object* data)
{
    auto visitor = data->asNodeVisitor();
    if (visitor->getVisitorType() != osg::NodeVisitor::UPDATE_VISITOR)
    {
        _listener->apply();
    }

    return traverse(node, data);
}

void ALListener::setGain(float v)
{
    _gain = v;
    dirty();
}

void ALListener::setVelocity(osg::Vec3 v)
{
    _velocity = v;
}

void ALListener::apply()
{
    auto transform = getWorldMatrices().front();

    auto translation = transform.getTrans();
    alListener3f(AL_POSITION, translation.x(), translation.y(), translation.z());
    AL_CHECK_ERROR;

    // get y axis as forward(at in openal), z axis as up
    // clang-format off
    float orientation[] =
    {
        static_cast<float>( transform( 1, 0 ) ),
        static_cast<float>( transform( 1, 1 ) ),
        static_cast<float>( transform( 1, 2 ) ),
        0, 0, 1
    };
    // clang-format on
    alListenerfv(AL_ORIENTATION, orientation);
    AL_CHECK_ERROR;

    alListenerfv(AL_VELOCITY, _velocity.ptr());
    AL_CHECK_ERROR;

    applyAttributes();
}

void ALListener::applyAttributes()
{
    if (!_dirty)
        return;

    alListenerf(AL_GAIN, _gain);
    AL_CHECK_ERROR;

    _dirty = false;
}

}  // namespace toy
