#ifndef TOY_LIGHTNING_H
#define TOY_LIGHTNING_H

#include <string>

#include <osg/Geometry>

namespace toy
{

// TODO create lines in gpu.
// Lines are generated by cpu, billboards are generated by gpu.
class Lightning : public osg::Geometry
{
public:
    // check lightning_billboard.geom for explanation
    enum billboard_type
    {
        bt_global = 1,
        bt_per_line = 2,
        bt_per_line_local = 3,
    };

    Lightning();

    void add(const std::string& pattern, const osg::Vec3& p0, const osg::Vec3& p1);

    void add(int segments, const osg::Vec3& p0, const osg::Vec3& p1);

    void resetLightning();

    float getBillboardWidth() const { return _billboardWidth; }
    void setBillboardWidth(float v);

    float getMaxJitter() const { return _maxJitter; }
    void setMaxJitter(float v) { _maxJitter = v; }

    float getMaxForkAngle() const { return _maxForkAngle; }
    void setMaxForkAngle(float v) { _maxForkAngle = v; }

    float getForkRate() const { return _forkRate; }
    void setForkRate(float v) { _forkRate = v; }

    const osg::Vec4& getCenterColor() const { return _centerColor; }
    void setCenterColor(const osg::Vec4& v);

    const osg::Vec4& getBorderColor() const { return _borderColor; }
    void setBorderColor(const osg::Vec4& v);

    bool getBillboard() const { return _billboard; }
    void setBillboard(bool v);

    float getExponent() const { return _exponent; }
    void setExponent(float v);

    billboard_type getBillboardType() const { return _billboardType; }
    void setBillboardType(billboard_type v);

    const osg::Vec3& getBillboardAxis() const { return _billboardAxis; }
    void setBillboardAxis(const osg::Vec3& v);

    void setAllBoltsStartEnd(const osg::Vec3& start, const osg::Vec3& end);

    bool getStable() const { return _stable; }
    void setStable(bool v) { _stable = v; }

    struct Bolt
    {
        std::string pattern;
        osg::Vec3 start;
        osg::Vec3 end;

        int getNumVertices() const;
    };

    using BoltList = std::vector<Bolt>;

    const BoltList& getBolts() const { return _bolts; }
    BoltList& getBolts() { return _bolts; }
    void setBolts(const BoltList& v) { _bolts = v; }

    static osg::Program* getBillboardProgram() { return _billboardProgram; }

private:
    std::vector<osg::Vec3> createLightning(const Bolt& bolt);

    void updateUniforms();

    bool _stable = true;
    bool _billboard = true;
    billboard_type _billboardType = bt_per_line_local;
    float _billboardWidth = 2;
    float _maxJitter = 0.5f;  // normalized, in size of subdivide
    float _maxForkAngle = osg::PI_4f;
    float _forkRate = 0.5f;
    float _exponent = 0.35f;

    osg::Vec3 _billboardAxis;  // for bt_global only
    osg::Vec4 _centerColor = osg::Vec4(1, 1, 1, 1);
    osg::Vec4 _borderColor = osg::Vec4(0, 0, 0.5, 1);

    static osg::ref_ptr<osg::Program> _billboardProgram;

    BoltList _bolts;

    osg::Vec3Array* _vertices;
};
}  // namespace toy

#endif  // TOY_LIGHTNING_H
