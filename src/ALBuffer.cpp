#include <ALBuffer.h>

#include <osgDB/FileUtils>
#include <AL.h>
#include <SoundObjectManager.h>

namespace toy
{

class ALBufferManager : public ALObjectManager
{
public:
    void deleteALObject(GLuint obj) override
    {
        alDeleteSources(1, &obj);
        AL_CHECK_ERROR;
    }
};

ALBuffer::ALBuffer() {}

ALBuffer::~ALBuffer()
{
    if (_alObject)
    {
        sgsom.get<ALBufferManager>()->scheduleGLObjectForDeletion(_alObject);
        _alObject = 0;
    }
}

ALBuffer::ALBuffer(const ALBuffer& src, const osg::CopyOp& copyop)
    : _bytes(src._bytes)
    , _channels(src._channels)
    , _bits(src._bits)
    , _frequency(src._frequency)
    , _data(src._data)
{
}

void ALBuffer::compileALObject()
{
    apply();
}

void ALBuffer::apply()
{
    if (!_alObject)
    {

        _alObject = alutCreateBufferFromFileImage(_data.data(), _data.size());
        ALUT_CHECK_ERROR;

        alGetBufferi(_alObject, AL_FREQUENCY, &_frequency);
        AL_CHECK_ERROR;
        alGetBufferi(_alObject, AL_SIZE, &_bytes);
        AL_CHECK_ERROR;
        alGetBufferi(_alObject, AL_CHANNELS, &_channels);
        AL_CHECK_ERROR;
        alGetBufferi(_alObject, AL_BITS, &_bits);
        AL_CHECK_ERROR;
    }
}

int ALBuffer::getSamples() const
{
    return _bytes * 8 / (_channels * _bits);
}

float ALBuffer::getDuration()
{
    compileALObject();
    return static_cast<float>(getSamples()) / _frequency;
}

}  // namespace toy
