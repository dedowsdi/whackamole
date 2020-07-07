#include <ToyCopyOp.h>

#include <ALBuffer.h>
#include <ALSource.h>

namespace toy
{
#define COPY_OP(TYPE, FLAG)                                                                \
    TYPE* ToyCopyOp::operator()(const TYPE* obj) const                                     \
    {                                                                                      \
        if (obj && _flags & FLAG)                                                          \
            return osg::clone(obj, *this);                                                 \
        else                                                                               \
            return const_cast<TYPE*>(obj);                                                 \
    }

COPY_OP(ALBuffer, ToyCopyOp::DEEP_COPY_AL_BUFFER)
COPY_OP(ALSource, ToyCopyOp::DEEP_COPY_AL_SOURCE)

ToyCopyOp::ToyCopyOp(CopyFlags flags) : osg::CopyOp(flags) {}

}  // namespace toy
