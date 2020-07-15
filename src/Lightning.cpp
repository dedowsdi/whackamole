#include "Lightning.h"

#include <algorithm>
#include <cassert>
#include <functional>
#include <random>
#include <vector>

#include <osg/BlendEquation>
#include <osg/BlendFunc>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/io_utils>
#include <osgDB/ReadFile>

#include <Game.h>
#include <Math.h>
#include <OsgFactory.h>

namespace toy
{

class LightningUpdater : public osg::Callback
{
public:
    LightningUpdater(Lightning& lightning) : _lightning(&lightning) {}

    bool run(osg::Object* object, osg::Object* data) override
    {
        _lightning->resetLightning();
        return traverse(object, data);
    }

private:
    Lightning* _lightning = 0;
};
osg::ref_ptr<osg::Program> Lightning::_billboardProgram;

Lightning::Lightning()
{
    setName("Lightning");

    // _geom = new osg::Geometry;
    setUseDisplayList(false);
    setDataVariance(osg::Object::DYNAMIC);
    addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 0));

    _vertices = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    setVertexArray(_vertices);

    setUpdateCallback(new LightningUpdater(*this));

    auto ss = getOrCreateStateSet();

    // Closing gap block lines behind it, disable depth write for blending. If
    // you need depth, you must write this again without color mask.
    ss->setAttributeAndModes(new osg::Depth(osg::Depth::LESS, 0, 1, false));
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
    ss->setAttributeAndModes(new osg::BlendFunc(GL_ONE, GL_ONE));
    ss->setAttributeAndModes(new osg::BlendEquation(osg::BlendEquation::RGBA_MAX));
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    setBillboardType(bt_per_line);
    setBillboard(true);
}

void Lightning::add(const std::string& pattern, const osg::Vec3& p0, const osg::Vec3& p1)
{
    _bolts.push_back(Bolt{pattern, p0, p1});
}

void Lightning::add(int segments, const osg::Vec3& p0, const osg::Vec3& p1)
{
    std::string s;
    std::generate_n(
        std::back_inserter(s), segments, []() { return unitRand() < 0.5 ? 'j' : 'f'; });
    add(s, p0, p1);
}

void Lightning::resetLightning()
{
    if ( !_vertices->empty() ) return;

    _vertices->clear();

    auto totalVeritces = std::accumulate(_bolts.begin(), _bolts.end(), 0ul,
        [](int sum, Bolt& b) { return sum + b.getNumVertices(); });

    _vertices->reserve(totalVeritces);

    for (auto& bolt: _bolts)
    {
        auto boltVertices = createLightning(bolt);
        std::copy(boltVertices.begin(), boltVertices.end(), std::back_inserter(*_vertices));
    }

    assert(_vertices->size() <= totalVeritces);

    static_cast<osg::DrawArrays*>(getPrimitiveSet(0))->setCount(_vertices->size());
    _vertices->dirty();
    dirtyBound();
}

void Lightning::setBillboardWidth(float v)
{
    _billboardWidth = v;
    getOrCreateStateSet()->addUniform(new osg::Uniform("billboard_width", v));
}

void Lightning::setCenterColor(const osg::Vec4& v)
{
    _centerColor = v;
    getOrCreateStateSet()->addUniform(new osg::Uniform("center_color", v));
}

void Lightning::setBorderColor(const osg::Vec4& v)
{
    _borderColor = v;
    getOrCreateStateSet()->addUniform(new osg::Uniform("border_color", v));
}

void Lightning::setBillboard(bool v)
{
    _billboard = v;
    auto ss = getOrCreateStateSet();
    if (_billboard)
    {
        setBillboardWidth(_billboardWidth);
        setCenterColor(_centerColor);
        setBorderColor(_borderColor);
        setExponent(_exponent);

        if (!_billboardProgram)
        {
            // render it as billboard, create faces along lines
            _billboardProgram = sgg.createProgram("shader/lightning_billboard.vert",
                "shader/lightning_billboard.geom", "shader/lightning_billboard.frag", GL_LINES,
                GL_TRIANGLE_STRIP, 12);
        }
        ss->setAttributeAndModes(_billboardProgram);

    }
    else
    {
        ss->removeAttribute(osg::StateAttribute::PROGRAM);
    }
}

void Lightning::setExponent(float v)
{
    _exponent = v;
    getOrCreateStateSet()->addUniform(new osg::Uniform("exponent", v));
}

void Lightning::setBillboardType(billboard_type v)
{
    _billboardType = v;
    getOrCreateStateSet()->setDefine("BILLBOARD_TYPE", std::to_string(_billboardType));

    if (v == bt_global)
    {
        setBillboardAxis(getBillboardAxis());
    }
}

void Lightning::setBillboardAxis(const osg::Vec3& v)
{
    _billboardAxis = v;
    getOrCreateStateSet()->addUniform(new osg::Uniform("billboard_axis", _billboardAxis));
}

void Lightning::setAllBoltsStartEnd(const osg::Vec3& start, const osg::Vec3& end)
{
    for (auto& bolt: _bolts)
    {
        bolt.start = start;
        bolt.end = end;
    }
}

std::vector<osg::Vec3> Lightning::createLightning(const Bolt& bolt)
{
    // subdivide bolt recursively with a ping pong vector :
    //      subdivide ping into pong, swap ping pong. The result is always in the ping.
    auto maxVertices = bolt.getNumVertices();
    auto ping = std::vector<osg::Vec3>({bolt.start, bolt.end});
    ping.reserve(maxVertices);
    auto pong = ping;

    float segmentLength = (bolt.end - bolt.start).length();
    if (segmentLength == 0)
    {
        OSG_WARN << "0 length bolt, ignred" << std::endl;
        return std::vector<osg::Vec3>{};
    }

    auto direction = bolt.end - bolt.start;
    direction /= segmentLength;

    for (auto letter: bolt.pattern)
    {
        if (letter != 'j' && letter != 'f')
            continue;

        segmentLength *= 0.5f;
        pong.clear();
        auto pongIter = std::back_inserter(pong);

        for (auto pingIter = ping.begin(); pingIter != ping.end();)
        {
            // jitter
            auto& p0 = *pingIter++;
            auto& p1 = *pingIter++;
            auto p2 = (p0 + p1) * 0.5f;
            auto v01 = p1 - p0;
            v01.normalize();

            auto v01Perp = randomOrghogonal(v01);
            v01Perp.normalize();
            p2 += v01Perp * unitRand() * _maxJitter * segmentLength;

            // segment 0
            *pongIter++ = p0;
            *pongIter++ = p2;

            // segment 1
            *pongIter++ = p2;
            *pongIter++ = p1;

            // fork
            // if (letter == 'f' && unitRand() < _forkRate)
            // {
            //     auto directionPerp = randomOrghogonal(direction);
            //     directionPerp.normalize();
            //     auto theta = linearRand(-_maxForkAngle, _maxForkAngle);
            //     auto forkDir = direction * cos(theta) + directionPerp * sin(theta);

            //     // segment 2
            //     *pongIter++ = p2;
            //     *pongIter++ = p2 + forkDir * segmentLength;
            // }
        }

        ping.swap(pong);
    }

    assert(ping.size() <= static_cast<unsigned>(maxVertices));
    return ping;
}

int Lightning::Bolt::getNumVertices() const
{
    auto numVertices = 2;
    for (auto& letter: pattern)
    {
        if (letter == 'j')
        {
            numVertices *= 2;
        }
        else if (letter == 'f')
        {
            numVertices *= 3;
        }
    }
    return numVertices;
}

}  // namespace toy
