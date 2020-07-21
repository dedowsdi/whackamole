#ifndef TT_OUTLINE_H
#define TT_OUTLINE_H

#include <osgFX/Effect>
#include <osgFX/Export>

namespace osg
{
class PolygonMode;
class CullFace;
}  // namespace osg

namespace toy
{

// Changed from osgFX::outline, support custom cullface and polygonmode
class Outline : public osgFX::Effect
{
public:
    /// Constructor.
    Outline();

    /// Copy constructor.
    Outline(const Outline& copy, const osg::CopyOp& op = osg::CopyOp::SHALLOW_COPY)
        : Effect(copy, op)
    {
        _width = copy._width;
        _color = copy._color;
        _technique = copy._technique;
    }

    // Effect class info
    META_Effect(osgFX, Outline, "Outline",
        "Stencil buffer based object outline effect.\n"
        "This effect needs a properly setup stencil buffer.",
        "Ulrich Hertlein");

    /// Set outline width.
    void setWidth(float w);

    /// Get outline width.
    float getWidth() const { return _width; }

    /// Set outline color.
    void setColor(const osg::Vec4& color);

    /// Get outline color.
    const osg::Vec4& getColor() const { return _color; }

    bool getEnableCullFace() const { return _enableCullFace; }
    void setEnableCullFace(bool v);

    int getCullFaceMode() const { return _cullFaceMode; }
    void setCullFaceMode(int v);

    int getPolygonModeFace() const { return _polygonModeFace; }
    void setPolygonModeFace(int v);

protected:
    /// Destructor.
    virtual ~Outline() {}

    /// Define available techniques.
    bool define_techniques();

private:
    bool _enableCullFace;

    int _cullFaceMode;

    int _polygonModeFace;

    /// Outline width.
    float _width;

    /// Outline color.
    osg::Vec4 _color;

    /// Technique.
    class OutlineTechnique;
    OutlineTechnique* _technique;
};

}  // namespace toy
#endif  // TT_OUTLINE_H
