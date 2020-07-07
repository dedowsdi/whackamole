#ifndef BOUNCINGBALL_ALBUFFER_H
#define BOUNCINGBALL_ALBUFFER_H

#include <memory>
#include <osg/Object>

namespace toy
{
class ALBuffer : public osg::Object
{
public:
    ALBuffer();

    ~ALBuffer();

    ALBuffer(const ALBuffer& src, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

    META_Object(toy, ALBuffer);

    void compileALObject();

    void apply();

    unsigned getALObject() { return _alObject; }

    // You must make sure it's already compiled if you call these functions;
    int getSamples() const;

    float getDuration();

    int getBytes() const { return _bytes; }

    int getChannels() const { return _channels; }

    int getBits() const { return _bits; }

    int getFrequency() const { return _frequency; }

    void setData(const std::vector<unsigned char>& data) { _data = data; }
    void setData(std::vector<unsigned char>&& data) { _data = move(data); }
    const std::vector<unsigned char>& getData() const { return _data; }
    std::vector<unsigned char>& getData() { return _data; }

private:
    unsigned _alObject = 0;

    int _bytes = 0;
    int _channels = 0;
    int _bits = 0;
    int _frequency = 0;

    std::vector<unsigned char> _data;
};

}  // namespace toy

#endif  // BOUNCINGBALL_ALBUFFER_H
