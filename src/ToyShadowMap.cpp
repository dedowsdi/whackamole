#include <ToyShadowMap.h>

#include <osgShadow/ShadowedScene>
#include <osg/ComputeBoundsVisitor>

#include <Config.h>
#include <Game.h>

#define IMPROVE_TEXGEN_PRECISION 1

namespace toy
{

void ToyShadowMap::cull(osgUtil::CullVisitor& cv)
{
    if (cv.getCurrentCamera() != _mainCamera)
    {
        _shadowedScene->osg::Group::traverse(cv);
        return;
    }

    // record the traversal mask on entry so we can reapply it later.
    unsigned int traversalMask = cv.getTraversalMask();

    osgUtil::RenderStage* orig_rs = cv.getRenderStage();

    // do traversal of shadow receiving scene which does need to be decorated by the shadow
    // map
    {
        cv.pushStateSet(_stateset.get());

        _shadowedScene->osg::Group::traverse(cv);

        cv.popStateSet();
    }

    // need to compute view frustum for RTT camera.
    // 1) get the light position
    // 2) get the center and extents of the view frustum

    const osg::Light* selectLight = 0;
    osg::Vec4 lightpos;
    osg::Vec3 lightDir;

    // MR testing giving a specific light
    osgUtil::PositionalStateContainer::AttrMatrixList& aml =
        orig_rs->getPositionalStateContainer()->getAttrMatrixList();
    for (osgUtil::PositionalStateContainer::AttrMatrixList::iterator itr = aml.begin();
         itr != aml.end(); ++itr)
    {
        const osg::Light* light = dynamic_cast<const osg::Light*>(itr->first.get());
        if (light)
        {
            if (_light.valid())
            {
                if (_light.get() == light)
                    selectLight = light;
                else
                    continue;
            }
            else
                selectLight = light;

            osg::RefMatrix* matrix = itr->second.get();
            if (matrix)
            {
                lightpos = light->getPosition() * (*matrix);
                lightDir = osg::Matrix::transform3x3(light->getDirection(), *matrix);
            }
            else
            {
                lightpos = light->getPosition();
                lightDir = light->getDirection();
            }
        }
    }

    osg::Matrix eyeToWorld;
    eyeToWorld.invert(*cv.getModelViewMatrix());

    lightpos = lightpos * eyeToWorld;
    lightDir = osg::Matrix::transform3x3(lightDir, eyeToWorld);
    lightDir.normalize();

    if (selectLight)
    {

        // set to ambient on light to black so that the ambient bias uniform can take it's
        // affect
        const_cast<osg::Light*>(selectLight)->setAmbient(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));

        // std::cout<<"----- VxOSG::ShadowMap selectLight spot cutoff
        // "<<selectLight->getSpotCutoff()<<std::endl;

        float fov = selectLight->getSpotCutoff() * 2;
        if (fov < 180.0f)  // spotlight, then we don't need the bounding box
        {
            osg::Vec3 position(lightpos.x(), lightpos.y(), lightpos.z());
            _camera->setProjectionMatrixAsPerspective(fov, 1.0, 0.1, 1000.0);
            _camera->setViewMatrixAsLookAt(
                position, position + lightDir, computeOrthogonalVector(lightDir));
        }
        else
        {
            // get the bounds of the model.
            osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
            cbbv.setTraversalMask(getShadowedScene()->getCastsShadowTraversalMask());

            _shadowedScene->osg::Group::traverse(cbbv);

            osg::BoundingBox bb = cbbv.getBoundingBox();

            if (lightpos[3] != 0.0)  // point light
            {
                osg::Vec3 position(lightpos.x(), lightpos.y(), lightpos.z());

                float centerDistance = (position - bb.center()).length();

                float znear = centerDistance - bb.radius();
                float zfar = centerDistance + bb.radius();
                float zNearRatio = 0.001f;
                if (znear < zfar * zNearRatio)
                    znear = zfar * zNearRatio;

                float top = (bb.radius() / centerDistance) * znear;
                float right = top;

                _camera->setProjectionMatrixAsFrustum(
                    -right, right, -top, top, znear, zfar);
                _camera->setViewMatrixAsLookAt(
                    position, bb.center(), computeOrthogonalVector(bb.center() - position));
            }
            else  // directional light
            {
                // make an orthographic projection
                osg::Vec3 ortho_lightDir(lightpos.x(), lightpos.y(), lightpos.z());
                auto radius = std::max(_projectionSize.x(), _projectionSize.y()) * 0.5f;
                osg::Vec3 position = _center + ortho_lightDir * radius * 2;

                float centerDistance = (position - _center).length();

                float znear = centerDistance - radius;
                float zfar = centerDistance + radius;
                float zNearRatio = 0.001f;
                if (znear < zfar * zNearRatio)
                    znear = zfar * zNearRatio;

                float top = _projectionSize.y() * 0.5f;
                float right = _projectionSize.x() * 0.5f;

                _camera->setProjectionMatrixAsOrtho(-right, right, -top, top, znear, zfar);
                _camera->setViewMatrixAsLookAt(
                    position, _center, computeOrthogonalVector(ortho_lightDir));
            }
        }

        cv.setTraversalMask(
            traversalMask & getShadowedScene()->getCastsShadowTraversalMask());

        // do RTT camera traversal
        _camera->accept(cv);

        _texgen->setMode(osg::TexGen::EYE_LINEAR);

#if IMPROVE_TEXGEN_PRECISION
        // compute the matrix which takes a vertex from local coords into tex coords
        // We actually use two matrices one used to define texgen
        // and second that will be used as modelview when appling to OpenGL
        auto ptsMatrix = _camera->getProjectionMatrix() *
                         osg::Matrix::translate(1.0, 1.0, 1.0) *
                         osg::Matrix::scale(0.5f, 0.5f, 0.5f);
        _texgen->setPlanesFromMatrix(ptsMatrix);

        // Place texgen with modelview which removes big offsets (making it float friendly)
        osg::RefMatrix* refMatrix =
            new osg::RefMatrix(_camera->getInverseViewMatrix() * *cv.getModelViewMatrix());

        cv.getRenderStage()->getPositionalStateContainer()->addPositionedTextureAttribute(
            _shadowTextureUnit, refMatrix, _texgen.get());

        osg::Matrixf m = osg::Matrix::inverse(*cv.getModelViewMatrix()) *
                         _camera->getViewMatrix() * ptsMatrix;
        _shadowMatrix->set(m);
#else
        // compute the matrix which takes a vertex from local coords into tex coords
        // will use this later to specify osg::TexGen..
        osg::Matrix MVPT = _camera->getViewMatrix() * _camera->getProjectionMatrix() *
                           osg::Matrix::translate(1.0, 1.0, 1.0) *
                           osg::Matrix::scale(0.5f, 0.5f, 0.5f);

        _texgen->setPlanesFromMatrix(MVPT);

        orig_rs->getPositionalStateContainer()->addPositionedTextureAttribute(
            _shadowTextureUnit, cv.getModelViewMatrix(), _texgen.get());
#endif
    }  // if(selectLight)

    // reapply the original traversal mask
    cv.setTraversalMask(traversalMask);
}

void ToyShadowMap::init()
{
    ShadowMap::init();

    // generate normal to deal with shadow acne
    auto texGen = new osg::TexGen;
    texGen->setMode(osg::TexGen::NORMAL_MAP);
    _stateset->setTextureAttributeAndModes(2, texGen);

    _shadowMatrix = new osg::Uniform("shadow_matrix", osg::Matrixf());
    _stateset->addUniform(_shadowMatrix);
    _stateset->setDefine("SHADOWED_SCENE", "1");
    _stateset->addUniform(
        new osg::Uniform("shadow_resolution", sgc.getVec2("scene.shadow.texture.size")));

    // override cast shadow program. If you need to customize it, you must use PROTECTED
    // for your program.
    auto cameraStateSet = _camera->getOrCreateStateSet();
    auto castShadow = sgg.createProgram("shader/cast_shadow.vert", "shader/cast_shadow.frag");
    cameraStateSet->setAttributeAndModes(
        castShadow, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    cameraStateSet->setDefine("CAST_SHADOW");
}

}  // namespace toy
