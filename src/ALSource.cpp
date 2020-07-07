#include <ALSource.h>

#include <cassert>

#include <osg/Group>
#include <osg/NodeVisitor>

#include <AL.h>
#include <ALBuffer.h>
#include <ALSource.h>
#include <SoundObjectManager.h>
#include <ToyCopyOp.h>

namespace toy
{

class ALSourceManager : public ALObjectManager
{
public:
    void deleteALObject(GLuint obj) override
    {
        alDeleteSources(1, &obj);
        AL_CHECK_ERROR;
    }
};

class ALSourceUpdater : public osg::Callback
{
public:
    ALSourceUpdater(ALSource* listener) : _source(listener) {}

    bool run(osg::Object* node, osg::Object* data) override
    {
        auto visitor = data->asNodeVisitor();
        if (visitor->getVisitorType() != osg::NodeVisitor::UPDATE_VISITOR)
        {
            _source->apply();
        }

        return traverse(node, data);
    }

private:
    ALSource* _source = 0;
};

ALSource::ALSource(ALBuffer* buf)
{
    setUpdateCallback(new ALSourceUpdater(this));
    setBuffer(buf);
}

ALSource::~ALSource()
{
    if (_alObject)
    {
        sgsom.get<ALSourceManager>()->scheduleGLObjectForDeletion(_alObject);
        _alObject = 0;
    }
}

ALSource::ALSource(const ALSource& src, const osg::CopyOp& copyop)
    : Node(src, copyop)
    , _loop(src._loop)
    , _relative(src._relative)
    , _coneInnerAngle(src._coneInnerAngle)
    , _coneOuterAngle(src._coneOuterAngle)
    , _coneOuterGain(src._coneOuterGain)
    , _gain(src._gain)
    , _maxDistance(src._maxDistance)
    , _maxGain(src._maxGain)
    , _minGain(src._minGain)
    , _pitch(src._pitch)
    , _referenceDistance(src._referenceDistance)
    , _rolloffFactor(src._rolloffFactor)
    , _direction(src._direction)
    , _position(src._position)
    , _velocity(src._velocity)
{
    _buffer = ToyCopyOp(copyop.getCopyFlags())(src._buffer);
    setReferenceDistance(1);
}

void ALSource::play()
{
    _play = true;
    _pause = false;
    _stop = false;
    _rewind = false;
}

void ALSource::pause()
{
    _play = false;
    _pause = true;
    _stop = false;
    _rewind = false;
}

void ALSource::stop()
{
    _play = false;
    _pause = false;
    _stop = true;
    _rewind = false;
}

void ALSource::rewind()
{
    _play = false;
    _pause = false;
    _stop = false;
    _rewind = true;
}

void ALSource::compileALObject()
{
    apply();
}

void ALSource::apply()
{
    if (!_buffer)
        return;

    if (!_alObject)
    {
        alGenSources(1, &_alObject);
        AL_CHECK_ERROR;
    }

    // apply stop and rewind before buffer change
    if (_stop)
    {
        alSourceStop(_alObject);
        AL_CHECK_ERROR;
        _stop = true;
    }

    if (_rewind)
    {
        alSourceRewind(_alObject);
        AL_CHECK_ERROR;
        _rewind = false;
    }

    if (_pause)
    {
        alSourcePause(_alObject);
        AL_CHECK_ERROR;
        _pause = false;
    }

    if (_dirtyBuffer)
    {
        _buffer->compileALObject();
        alSourcei(_alObject, AL_BUFFER, _buffer->getALObject());
        AL_CHECK_ERROR;
        _dirtyBuffer = false;
    }

    auto transform = getWorldMatrices().front();

    auto direction = _direction == osg::Vec3f()
                         ? _direction
                         : osg::Matrix::transform3x3(_direction, transform);
    alSourcefv(_alObject, AL_DIRECTION, direction.ptr());
    AL_CHECK_ERROR;

    auto pos = transform.getTrans();
    alSource3f(_alObject, AL_POSITION, pos.x(), pos.y(), pos.z());
    AL_CHECK_ERROR;

    // TODO get velocity from bullet if possible
    alSourcefv(_alObject, AL_VELOCITY, _velocity.ptr());
    AL_CHECK_ERROR;

    applyAttributes();

    // apply play in the end
    if (_play)
    {
        alSourcePlay(_alObject);
        AL_CHECK_ERROR;
        _play = false;
    }
}

void ALSource::applyAttributes()
{
    if (!_dirtyAttributes)
        return;

    alSourcef(_alObject, AL_GAIN, _gain);
    AL_CHECK_ERROR;
    alSourcei(_alObject, AL_SOURCE_RELATIVE, _relative);
    AL_CHECK_ERROR;
    alSourcei(_alObject, AL_LOOPING, _loop);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_MIN_GAIN, _minGain);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_MAX_GAIN, _maxGain);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_REFERENCE_DISTANCE, _referenceDistance);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_ROLLOFF_FACTOR, _rolloffFactor);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_MAX_DISTANCE, _maxDistance);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_PITCH, _pitch);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_CONE_INNER_ANGLE, _coneInnerAngle);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_CONE_OUTER_ANGLE, _coneOuterAngle);
    AL_CHECK_ERROR;
    alSourcef(_alObject, AL_CONE_OUTER_GAIN, _coneOuterGain);
    AL_CHECK_ERROR;

    _dirtyAttributes = false;
}

float ALSource::getBufferDuration()
{
    apply();
    return _buffer ? _buffer->getDuration() : 0;
}

void ALSource::setBuffer(ALBuffer* buffer)
{
    _buffer = buffer;
    dirtyBuffer();
}

void ALSource::setLoop(bool v)
{
    _loop = v;
    dirtyAttributes();
}

void ALSource::setRelative(bool v)
{
    _relative = v;
    dirtyAttributes();
}

void ALSource::setConeInnerAngle(float v)
{
    _coneInnerAngle = v;
    dirtyAttributes();
}

void ALSource::setConeOuterAngle(float v)
{
    _coneOuterAngle = v;
    dirtyAttributes();
}

void ALSource::setConeOuterGain(float v)
{
    _coneOuterGain = v;
    dirtyAttributes();
}

void ALSource::setGain(float v)
{
    _gain = v;
    dirtyAttributes();
}

void ALSource::setMaxDistance(float v)
{
    _maxDistance = v;
    dirtyAttributes();
}

void ALSource::setMaxGain(float v)
{
    _maxGain = v;
    dirtyAttributes();
}

void ALSource::setMinGain(float v)
{
    _minGain = v;
    dirtyAttributes();
}

void ALSource::setPitch(float v)
{
    _pitch = v;
    dirtyAttributes();
}

void ALSource::setReferenceDistance(float v)
{
    _referenceDistance = v;
    dirtyAttributes();
}

void ALSource::setRolloffFactor(float v)
{
    _rolloffFactor = v;
    dirtyAttributes();
}

void playSound(osg::Group* node, ALSource* sound, bool removeAfterFinshed)
{
    assert(node);
    assert(sound);

    node->addChild(sound);
    sound->play();

    if (removeAfterFinshed && !sound->getLoop())
    {
        node->addUpdateCallback(
            new RemoveSoundCallback(sound, true, sound->getBufferDuration()));
    }
}

bool RemoveSoundCallback::run(osg::Object* object, osg::Object* data)
{
    auto visitor = data->asNodeVisitor();
    if (visitor && visitor->asUpdateVisitor())
    {
        auto currentTime = visitor->getFrameStamp()->getReferenceTime();
        if (_lastTime == 0)
        {
            _lastTime = currentTime;
        }
        _time -= currentTime - _lastTime;
        _lastTime = currentTime;

        if (_time <= 0)
        {
            auto node = dynamic_cast<osg::Group*>(object);
            osg::ref_ptr<ALSource> sound;
            if (_sound.lock(sound))
            {
                if (_stopSound)
                {
                    _sound->stop();
                }
                node->removeChild(sound);
                OSG_DEBUG << "Remove sound " << this << std::endl;
            }

            auto thisCallback = osg::ref_ptr<osg::Callback>(this);
            node->removeUpdateCallback(this);
            return thisCallback->traverse(object, data);
        }
    }

    return traverse(object, data);
}

}  // namespace toy
