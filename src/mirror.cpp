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

NORI_NAMESPACE_BEGIN

/// Ideal mirror BRDF
class Mirror : public BSDF {
public:
    Mirror(const PropertyList &) { }

    Color3f eval(const BSDFParams &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFParams &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    BSDFRecord sample(const Vector3f &wi, const Point2f &) const {
        BSDFRecord bRec;

        // Check if wi is in same hemisphere as surface normal,
        // otherwise return black
        if (Frame::cosTheta(wi) <= 0)
            return bRec;
        
        // Reflect
        bRec.params.wi = wi;
        bRec.params.wo = Vector3f(-wi.x(), -wi.y(), wi.z());

        // Mirror is a discrete BRDF
        bRec.measure = EDiscrete;

        // Reflected light retains all of its radiance
        bRec.value = Color3f(1.0f);

        return bRec;
    }

    std::string toString() const {
        return "Mirror[]";
    }
};

NORI_REGISTER_CLASS(Mirror, "mirror");
NORI_NAMESPACE_END
