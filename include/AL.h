#ifndef BOUNCINGBALL_AL_H
#define BOUNCINGBALL_AL_H

#include <iostream>

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alut.h>

#define AL_CHECK_PER_CALL

#ifdef AL_CHECK_PER_CALL

#    define AL_CHECK_ERROR                                                                 \
        {                                                                                  \
            auto error = alGetError();                                                     \
            if (error != AL_NO_ERROR)                                                      \
            {                                                                              \
                std::cerr << "OpenAL error:" << __FILE__ << ":" << __FUNCTION__ << ":"     \
                          << __LINE__ << ":" << alErrorToString(error) << std::endl;       \
            }                                                                              \
        }

#    define ALUT_CHECK_ERROR                                                               \
        {                                                                                  \
            auto error = alutGetError();                                                   \
            if (error != ALUT_ERROR_NO_ERROR)                                              \
            {                                                                              \
                std::cerr << "OpenAL alut error:" << __FILE__ << ":" << __FUNCTION__       \
                          << ":" << __LINE__ << ":" << alutGetErrorString(error)           \
                          << std::endl;                                                    \
            }                                                                              \
        }

#else

#    define AL_CHECK_ERROR
#    define ALUT_CHECK_ERROR

#endif /* ifndef AL_CHECK_PER_CALL */

namespace toy
{
const char* alErrorToString(int error);

const char* alDistanceModelToString(int model);

bool initOpenAL(int argc, char* argv[]);

void closeOpenAL();

}  // namespace toy

#endif  // BOUNCINGBALL_AL_H
