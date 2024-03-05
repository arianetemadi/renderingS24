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

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &) const {
        throw NoriException("Unimplemented!");

        // TODO: Check if bRec.wi is in same hemisphere as surface 
        // normal, otherwise return black
        // TODO: Generate a sample of the BRDF's function into bRec.wo

        // Can use these values: mirror is a discrete BRDF and the 
        // index of refraction eta doesn't change because the ray is
        // reflected and does not enter a new medium.
        bRec.measure = EDiscrete;
        bRec.eta = 1.0f;

        // TODO: Return correct BRDF value for drawn sample
    }

    std::string toString() const {
        return "Mirror[]";
    }
};

NORI_REGISTER_CLASS(Mirror, "mirror");
NORI_NAMESPACE_END
