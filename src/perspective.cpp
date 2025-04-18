/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/camera.h>
#include <nori/rfilter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

/**
 * \brief Perspective camera with depth of field
 *
 * This class implements a simple perspective camera model. It uses an
 * infinitesimally small aperture, creating an infinite depth of field.
 */
class PerspectiveCamera : public Camera {
public:
    PerspectiveCamera(const PropertyList &propList) {
        /* Width and height in pixels. Default: 720p */
        m_outputSize.x() = propList.getInteger("width", 1280);
        m_outputSize.y() = propList.getInteger("height", 720);
        m_invOutputSize = m_outputSize.cast<float>().cwiseInverse();

        /* Specifies an optional camera-to-world transformation. Default: none */
        m_cameraToWorld = propList.getTransform("toWorld", Transform());

        // /* Specify camera velocity for motion blur */
        // m_cameraToWorldOriginal = m_cameraToWorld;
        // motionBlur = propList.getBoolean("motionBlur", false);
        // velocity = propList.getVector("motionBlurVelocity", Vector3f(0.f));

        /* Horizontal field of view in degrees */
        m_fov = propList.getFloat("fov", 30.0f);

        /* Near and far clipping planes in world-space units */
        m_nearClip = propList.getFloat("nearClip", 1e-4f);
        m_farClip = propList.getFloat("farClip", 1e4f);

        // /* Depth of Field (DoF) */
        // DoF = propList.getBoolean("DoF", false);
        // aperture = propList.getFloat("aperture", 100);
        // focalLength = propList.getFloat("focalLength", 900);

        m_rfilter = NULL;
    }

    void activate() {
        float aspect = m_outputSize.x() / (float) m_outputSize.y();

        /* Project vectors in camera space onto a plane at z=1:
         *
         *  xProj = cot * x / z
         *  yProj = cot * y / z
         *  zProj = (far * (z - near)) / (z * (far-near))
         *  The cotangent factor ensures that the field of view is 
         *  mapped to the interval [-1, 1].
         */
        float recip = 1.0f / (m_farClip - m_nearClip),
              cot = 1.0f / std::tan(degToRad(m_fov / 2.0f));

        Eigen::Matrix4f perspective;
        perspective <<
            cot, 0,   0,   0,
            0, cot,   0,   0,
            0,   0,   m_farClip * recip, -m_nearClip * m_farClip * recip,
            0,   0,   1,   0;

        /**
         * Translation and scaling to shift the clip coordinates into the
         * range from zero to one. Also takes the aspect ratio into account.
         */
        m_sampleToCamera = Transform( 
            Eigen::DiagonalMatrix<float, 3>(Vector3f(-0.5f, -0.5f * aspect, 1.0f)) *
            Eigen::Translation<float, 3>(-1.0f, -1.0f/aspect, 0.0f) * perspective).inverse();

        /* If no reconstruction filter was assigned, instantiate a Gaussian filter */
        if (!m_rfilter)
            m_rfilter = static_cast<ReconstructionFilter *>(
                NoriObjectFactory::createInstance("box", PropertyList()));
    }

    Color3f sampleRay(Ray3f &ray,
            const Point2f &samplePosition,
            const Point2f &apertureSample) const {
        /* Compute the corresponding position on the 
           near plane (in local camera space) */
        Point3f nearP = m_sampleToCamera * Point3f(
            samplePosition.x() * m_invOutputSize.x(),
            samplePosition.y() * m_invOutputSize.y(), 0.0f);

        /* Turn into a normalized ray direction, and
           adjust the ray interval accordingly */
        Vector3f d = nearP.normalized();
        float invZ = 1.0f / d.z();

        ray.o = m_cameraToWorld * Point3f(0, 0, 0);
        ray.d = m_cameraToWorld * d;
        ray.mint = m_nearClip * invZ;
        ray.maxt = m_farClip * invZ;
        ray.update();

        // /* Depth of Field */
        // if (DoF) {
        //     // find the camera direction
        //     Point3f np22 = m_sampleToCamera * Point3f(0.5, 0.5, 0);  // middle pixel
        //     Vector3f cam_dir = np22.normalized();
        //     cam_dir = m_cameraToWorld * cam_dir;

        //     d = ray.d;

        //     // new origin
        //     Vector3f o = m_cameraToWorld * Point3f(aperture * apertureSample(0), aperture * apertureSample(1), 0);
        //     ray.o = o;

        //     // new direction
        //     Vector3f a = o - m_cameraToWorld * Point3f(0, 0, 0);
        //     float t = focalLength / cam_dir.dot(d);
        //     ray.d = t * d - a;

        //     ray.update();
        // }

        return Color3f(1.0f);
    }

    void addChild(NoriObject *obj) {
        switch (obj->getClassType()) {
            case EReconstructionFilter:
                if (m_rfilter)
                    throw NoriException("Camera: tried to register multiple reconstruction filters!");
                m_rfilter = static_cast<ReconstructionFilter *>(obj);
                break;

            default:
                throw NoriException("Camera::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }

    // For motion blur
    // TODO: refactor the boolean variable?
    // void animate(const float &time) {
    //     if (motionBlur) {
    //         Eigen::Matrix4f translationMatrix;
    //         Vector3f translation = time * velocity;
    //         translationMatrix <<
    //             1, 0, 0, translation(0),
    //             0, 1, 0, translation(1),
    //             0, 0, 1, translation(2),
    //             0, 0, 0, 1;
    //         m_cameraToWorld = Transform(translationMatrix) * m_cameraToWorldOriginal;
    //     }
    // }

    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "PerspectiveCamera[\n"
            "  cameraToWorld = %s,\n"
            "  outputSize = %s,\n"
            "  fov = %f,\n"
            "  clip = [%f, %f],\n"
            "  rfilter = %s\n"
            "]",
            indent(m_cameraToWorld.toString(), 18),
            m_outputSize.toString(),
            m_fov,
            m_nearClip,
            m_farClip,
            indent(m_rfilter->toString())
        );
    }
private:
    Vector2f m_invOutputSize;
    Transform m_sampleToCamera;
    Transform m_cameraToWorld;
    float m_fov;
    float m_nearClip;
    float m_farClip;

    // /* For Motion Blur */
    // bool motionBlur;
    // Transform m_cameraToWorldOriginal;  // Original camera transformation
    // Vector3f velocity;  // Camera velocity

    // /* For Depth of Field (DoF) */
    // bool DoF;
    // float aperture;
    // float focalLength;
};

NORI_REGISTER_CLASS(PerspectiveCamera, "perspective");
NORI_NAMESPACE_END
