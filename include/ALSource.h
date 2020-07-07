#ifndef BOUNCINGBALL_ALSOURCE_H
#define BOUNCINGBALL_ALSOURCE_H

#include <climits>

#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/observer_ptr>

namespace toy
{
class ALBuffer;

// Use world position as source position
// Use zero as direction if it's zero, otherwise use direction * world_rotation
// It's applied in update traversal.
class ALSource : public osg::Node
{
public:
    using BufferList = std::vector<osg::ref_ptr<ALBuffer>>;

    ALSource(ALBuffer* buf = 0);

    ~ALSource();

    ALSource(const ALSource& src, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

    META_Node(toy, ALSource);

    void play();

    void pause();

    void stop();

    void rewind();

    void compileALObject();

    void apply();

    void applyAttributes();

    float getBufferDuration();

    // be careful with this. It's an invalid operation if you change buffer during playing.
    void setBuffer(ALBuffer* _buffer);
    ALBuffer* getBuffer() { return _buffer; }

    bool getLoop() const { return _loop; }
    void setLoop(bool v);

    bool getRelative() const { return _relative; }
    void setRelative(bool v);

    float getConeInnerAngle() const { return _coneInnerAngle; }
    void setConeInnerAngle(float v);

    float getConeOuterAngle() const { return _coneOuterAngle; }
    void setConeOuterAngle(float v);

    float getConeOuterGain() const { return _coneOuterGain; }
    void setConeOuterGain(float v);

    float getGain() const { return _gain; }
    void setGain(float v);

    float getMaxDistance() const { return _maxDistance; }
    void setMaxDistance(float v);

    float getMaxGain() const { return _maxGain; }
    void setMaxGain(float v);

    float getMinGain() const { return _minGain; }
    void setMinGain(float v);

    float getPitch() const { return _pitch; }
    void setPitch(float v);

    float getReferenceDistance() const { return _referenceDistance; }
    void setReferenceDistance(float v);

    float getRolloffFactor() const { return _rolloffFactor; }
    void setRolloffFactor(float v);

    unsigned getALObject() const { return _alObject; }

private:
    void dirtyAttributes() { _dirtyAttributes = true; }
    void dirtyBuffer() { _dirtyBuffer = true; }

    bool _dirtyAttributes = false;
    bool _dirtyBuffer = false;

    // only 1 of 4 can be true
    bool _play = false;
    bool _pause = false;
    bool _stop = false;
    bool _rewind = false;

    unsigned _alObject = 0;
    osg::ref_ptr<ALBuffer> _buffer;

    // TODO missing type, buffer queue, buffer process, sec offset, sampler offset,
    // byte offset
    bool _loop = false;
    bool _relative = false;
    float _coneInnerAngle = 360.0f;
    float _coneOuterAngle = 360.0f;
    float _coneOuterGain = 1.0f;
    float _gain = 1;
    float _maxDistance = FLT_MAX;
    float _maxGain = 1.0f;
    float _minGain = 0.0f;
    float _pitch = 1.0f;
    float _referenceDistance = 1.0f;
    float _rolloffFactor = 1.0f;
    osg::Vec3f _direction;
    osg::Vec3f _position;
    osg::Vec3f _velocity;
};

// removeAfterFinished only work for non loop sound source.
void playSound(osg::Group* node, ALSource* sound, bool removeAfterFinshed = true);

// stop, remove sound after some time
class RemoveSoundCallback : public osg::Callback
{
public:
    RemoveSoundCallback(ALSource* sound, bool stopSound, float time)
        : _sound(sound), _stopSound(stopSound), _time(time)
    {
    }

    bool run(osg::Object* object, osg::Object* data) override;

private:
    bool _stopSound = true;
    float _time = 0;
    float _lastTime = 0;
    osg::observer_ptr<ALSource> _sound;
};

}  // namespace toy

#endif  // BOUNCINGBALL_ALSOURCE_H
