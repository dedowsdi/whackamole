#include "Outline.h"

#include <osg/CullFace>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/PolygonMode>
#include <osg/Stencil>
#include <osg/Texture1D>

namespace
{
const unsigned int Override_On = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;
const unsigned int Override_Off = osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE;
}  // namespace

namespace toy
{
/// Register prototype.
// osgFX::Registry::Proxy proxy(new Outline);

class Outline::OutlineTechnique : public osgFX::Technique
{
public:
    OutlineTechnique()
        : Technique()
        , _lineWidth()
        , _width(2)
        , _material()
        , _color(1, 1, 1, 1)
        , _cullFace(new osg::CullFace(osg::CullFace::FRONT))
        , _polygonMode(new osg::PolygonMode(osg::PolygonMode::BACK, osg::PolygonMode::LINE))
    {
    }

    /// Validate.
    bool validate(osg::State&) const { return true; }

    /// Set outline width.
    void setWidth(float w)
    {
        _width = w;
        if (_lineWidth.valid())
        {
            _lineWidth->setWidth(w);
        }
    }

    void setEnableCullFace(bool v) { _enableCullFace = v; }

    void setCullFaceMode(osg::CullFace::Mode v) { _cullFace->setMode(v); }

    void setPolygonModeFace(osg::PolygonMode::Face v)
    {
        _polygonMode->setMode(v, osg::PolygonMode::LINE);
    }

    /// Set outline color.
    void setColor(const osg::Vec4& color)
    {
        _color = color;
        if (_material.valid())
        {
            const osg::Material::Face face = osg::Material::FRONT_AND_BACK;
            _material->setAmbient(face, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
            _material->setDiffuse(face, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
            _material->setSpecular(face, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
            _material->setEmission(face, color);
        }
    }

protected:
    /// Define render passes.
    void define_passes()
    {

        /*
         * draw
         * - set stencil buffer to ref=1 where draw occurs
         * - clear stencil buffer to 0 where test fails
         */
        {
            osg::StateSet* state = new osg::StateSet;

            // stencil op
            osg::Stencil* stencil = new osg::Stencil;
            stencil->setFunction(osg::Stencil::ALWAYS, 1, ~0u);
            stencil->setOperation(
                osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::REPLACE);
            state->setAttributeAndModes(stencil, Override_On);

            addPass(state);
        }

        /*
         * post-draw
         * - only draw where draw didn't set the stencil buffer
         * - draw only back-facing polygons
         * - draw back-facing polys as lines
         * - disable depth-test, lighting & texture
         */
        {
            osg::StateSet* state = new osg::StateSet;

            // stencil op
            osg::Stencil* stencil = new osg::Stencil;
            stencil->setFunction(osg::Stencil::NOTEQUAL, 1, ~0u);
            stencil->setOperation(
                osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::REPLACE);
            state->setAttributeAndModes(stencil, Override_On);

            // cull front-facing polys
            if (_enableCullFace)
            {
                OSG_NOTICE << "enabled" << std::endl;
                state->setAttributeAndModes(_cullFace, Override_On);
            }

            state->setAttributeAndModes(_polygonMode);

            // outline width
            _lineWidth = new osg::LineWidth;
            setWidth(_width);
            state->setAttributeAndModes(_lineWidth.get(), Override_On);

            // outline color/material
            _material = new osg::Material;
            _material->setColorMode(osg::Material::OFF);
            setColor(_color);
            state->setAttributeAndModes(_material.get(), Override_On);

            // disable modes
            state->setMode(GL_BLEND, Override_Off);
            // state->setMode(GL_DEPTH_TEST, Override_Off);
            state->setTextureMode(0, GL_TEXTURE_1D, Override_Off);
            state->setTextureMode(0, GL_TEXTURE_2D, Override_Off);
            state->setTextureMode(0, GL_TEXTURE_3D, Override_Off);

            addPass(state);
        }
    }

private:
    /// Outline width.
    osg::ref_ptr<osg::LineWidth> _lineWidth;
    float _width;

    /// Outline Material.
    osg::ref_ptr<osg::Material> _material;
    osg::Vec4 _color;

    bool _enableCullFace;
    osg::ref_ptr<osg::CullFace> _cullFace;

    osg::ref_ptr<osg::PolygonMode> _polygonMode;
};

/**
 * Outline effect.
 */
Outline::Outline() : Effect(), _width(2), _color(1, 1, 1, 1), _technique(0) {}

void Outline::setWidth(float w)
{
    _width = w;
    if (_technique)
    {
        _technique->setWidth(w);
    }
}

void Outline::setColor(const osg::Vec4& color)
{
    _color = color;
    if (_technique)
    {
        _technique->setColor(color);
    }
}

void Outline::setEnableCullFace(bool v)
{
    _enableCullFace = v;
    if (_technique)
    {
        _technique->setEnableCullFace(v);
    }
}

void Outline::setCullFaceMode(int v)
{
    _cullFaceMode = v;
    if (_technique)
    {
        _technique->setCullFaceMode(static_cast<osg::CullFace::Mode>(v));
    }
}

void Outline::setPolygonModeFace(int v)
{
    _polygonModeFace = v;
    if (_technique)
    {
        _technique->setPolygonModeFace(static_cast<osg::PolygonMode::Face>(v));
    }
}

bool Outline::define_techniques()
{
    _technique = new OutlineTechnique;
    addTechnique(_technique);

    setWidth(_width);
    setColor(_color);
    setEnableCullFace(_enableCullFace);
    setCullFaceMode(_cullFaceMode);
    setPolygonModeFace(_polygonModeFace);

    return true;
}
}  // namespace toy
