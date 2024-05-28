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

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <cmath>

NORI_NAMESPACE_BEGIN

/**
 * \brief Diffuse / Lambertian BRDF model
 */
class Diffuse : public BSDF {
public:
    Diffuse(const PropertyList &propList) {
        m_albedo = propList.getColor("albedo", Color3f(0.5f));
        useCosine = propList.getBoolean("use_cosine", false);
    }

    /// Evaluate the BRDF model
    Color3f eval(const BSDFParams &params) const {
        /* This is a smooth BRDF -- return zero when queried 
           for illumination on the backside */
        if (Frame::cosTheta(params.wi) <= 0 || 
            Frame::cosTheta(params.wo) <= 0)
            return Color3f(0.0f);

        /* The BRDF is simply the albedo / pi */
        return m_albedo * INV_PI;
    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFParams &params) const {
        /* This is a smooth BRDF -- return zero when queried 
           for illumination on the backside.
           
           Note that the directions in 'params' are in local coordinates.*/
        if (Frame::cosTheta(params.wi) <= 0 ||
            Frame::cosTheta(params.wo) <= 0)
            return 0.0f;

        if (useCosine) {
            return Warp::squareToCosineHemispherePdf(params.wo);
        } else {
            return Warp::squareToUniformHemispherePdf(params.wo);
        }
    }

    /// Draw a sample from the BRDF model
    BSDFRecord sample(const Vector3f &wi, const Point2f &sample) const {
        BSDFRecord bRec;
        bRec.params.wi = wi;
        bRec.measure = ESolidAngle;

        /* Warp a uniformly distributed sample on [0,1]^2
           to a direction on the hemisphere */
        if (useCosine) {
            bRec.params.wo = Warp::squareToCosineHemisphere(sample);
        } else {
            bRec.params.wo = Warp::squareToUniformHemisphere(sample);
        }

        if (Frame::cosTheta(wi) <= 0 ||
            Frame::cosTheta(bRec.params.wo) <= 0)
            return bRec;

        // for cosine sampling, the value can be set directly to m_albedo,
        // but I didn't do it to prioritize code readability
        bRec.value = eval(bRec.params) * Frame::cosTheta(bRec.params.wo) / pdf(bRec.params);

        return bRec;
    }

    bool isDiffuse() const {
        return true;
    }

    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "Diffuse[\n"
            "  albedo = %s\n"
            "]", m_albedo.toString());
    }

    EClassType getClassType() const { return EBSDF; }
private:
    Color3f m_albedo;
    bool useCosine;
};

NORI_REGISTER_CLASS(Diffuse, "diffuse");
NORI_NAMESPACE_END
