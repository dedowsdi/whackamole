#ifndef BOUNCINGBALL_TOYCOPYOP_H
#define BOUNCINGBALL_TOYCOPYOP_H

#include <osg/CopyOp>

namespace toy
{

class ALBuffer;
class ALSource;

class ToyCopyOp : public osg::CopyOp
{
public:
    enum toy
    {
        DEEP_COPY_AL_BUFFER = 1 << 16,
        DEEP_COPY_AL_SOURCE = 1 << 17
    };

    ToyCopyOp(CopyFlags flags = SHALLOW_COPY);

    virtual ALBuffer* operator()(const ALBuffer* v) const;
    virtual ALSource* operator()(const ALSource* v) const;
};

}  // namespace toy

#endif  // BOUNCINGBALL_TOYCOPYOP_H
