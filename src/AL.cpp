#include <AL.h>

#include <cstring>
#include <string>

#include <osg/Notify>

namespace toy
{

const char* alErrorToString(int error)
{
    switch (error)
    {
        case AL_NO_ERROR:
            return "AL_NO_ERROR";
        case AL_INVALID_NAME:
            return "AL_INVALID_NAME";
        case AL_INVALID_ENUM:
            return "AL_INVALID_ENUM";
        case AL_INVALID_VALUE:
            return "AL_INVALID_VALUE";
        case AL_INVALID_OPERATION:
            return "AL_INVALID_OPERATION";
        case AL_OUT_OF_MEMORY:
            return "AL_OUT_OF_MEMORY";
        default:
            return "";
    }
}

const char* alDistanceModelToString(int model)
{
    switch (model)
    {
        case AL_NONE:
            return "AL_INVERSE_DISTANCE";
        case AL_INVERSE_DISTANCE:
            return "AL_INVERSE_DISTANCE";
        case AL_INVERSE_DISTANCE_CLAMPED:
            return "AL_INVERSE_DISTANCE_CLAMPED";
        case AL_LINEAR_DISTANCE:
            return "AL_LINEAR_DISTANCE";
        case AL_LINEAR_DISTANCE_CLAMPED:
            return "AL_LINEAR_DISTANCE_CLAMPED";
        case AL_EXPONENT_DISTANCE:
            return "AL_EXPONENT_DISTANCE";
        case AL_EXPONENT_DISTANCE_CLAMPED:
            return "AL_EXPONENT_DISTANCE_CLAMPED";
        default:
            return "";
    }
}

void listAudioDevices(const ALCchar* devices)
{
    if (!devices || !*devices)
    {
        OSG_NOTICE << "empty device string!!!" << std::endl;
        return;
    }

    auto device = devices;

    OSG_NOTICE << "OpenAL audio device list:\n";
    while (*device)
    {
        OSG_NOTICE << device << "\n";
        device += (strlen(device) + 1);
    }
    OSG_NOTICE << std::endl;
}

bool initOpenAL(int argc, char* argv[])
{
    if (alcIsExtensionPresent(NULL, "ALC_ENUMERATION_EXT") == AL_FALSE)
    {
        OSG_WARN << "OpenAL : enumeration extension not available\n" << std::endl;
    }
    else
    {
        listAudioDevices(alcGetString(NULL, ALC_DEVICE_SPECIFIER));
    }

    auto defaultDeviceName = alcGetString(NULL, ALC_DEFAULT_DEVICE_SPECIFIER);
    OSG_NOTICE << "default OpenAL device : " << defaultDeviceName << std::endl;

    alutInit(&argc, argv);
    ALUT_CHECK_ERROR

    auto context = alcGetCurrentContext();
    auto device = alcGetContextsDevice(context);
    OSG_NOTICE << "OpenAL open device : " << alcGetString(device, ALC_DEVICE_SPECIFIER)
               << std::endl;

    OSG_NOTICE << "OpenAL distance model : "
               << alDistanceModelToString(alGetInteger(AL_DISTANCE_MODEL)) << std::endl;
    return true;
}

void closeOpenAL()
{
    alutExit();
    ALUT_CHECK_ERROR;
}

}  // namespace toy
