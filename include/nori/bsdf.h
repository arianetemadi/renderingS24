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

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Convenience data structure holding all parameters passed
 * for evaluation methods of the \ref BSDF
 */
struct BSDFParams {
    /// Incident direction (in the local frame)
    Vector3f wi;

    /// Outgoing direction (in the local frame)
    Vector3f wo;

    /// Position on the surface (expressed in terms of uv coordinates)
    Point2f uv { 0.f, 0.f };
};

/**
 * \brief Convenience data structure used for returning of multiple
 * results from sampling routine in \ref BSDF
 */
struct BSDFRecord {
    /// Record of specific sampling configuration
    BSDFParams params;

    /// Measure associated with the sample
    EMeasure measure;

    /// The BSDF value divided by the probability density of the sample
    Color3f value { 0.f, 0.f, 0.f };
};

/**
 * \brief Superclass of all bidirectional scattering distribution functions
 */
class BSDF : public NoriObject {
public:
    /**
     * \brief Sample the BSDF and return the importance weight (i.e. the
     * value of the BSDF * cos(theta_o) divided by the probability density
     * of the sample with respect to solid angles), and other record data.
     *
     * \param wi      The incident direction in the local frame
     * \param sample  A uniformly distributed sample on \f$[0,1]^2\f$
     *
     * \return BSDF record structure. Includes:
     *         1) The direction of the ingoing/outgoing ray in the local frame.
     *         2) UV location of the sampled point on the surface (if applicable)
     *         3) The BSDF value divided by the probability density of the 
     *         sample. The returned value also includes the cosine
     *         foreshortening factor associated with the outgoing direction,
     *         when this is appropriate. A zero value means that sampling
     *         failed.
     *         4) The measure associated with the sample
     */
    virtual BSDFRecord sample(const Vector3f &wi, const Point2f &sample) const = 0;

    /**
     * \brief Evaluate the BSDF for a pair of directions and measure
     * specified in \code bRec
     *
     * \param bRec
     *     A record with detailed information on the BSDF query
     * \return
     *     The BSDF value, evaluated for each color channel
     */
    virtual Color3f eval(const BSDFParams &params) const = 0;

    /**
     * \brief Compute the probability of sampling \c bRec.wo
     * (conditioned on \c bRec.wi).
     *
     * This method provides access to the probability density that
     * is realized by the \ref sample() method.
     *
     * \param bRec
     *     A record with detailed information on the BSDF query
     *
     * \return
     *     A probability/density value expressed with respect
     *     to the specified measure
     */

    virtual float pdf(const BSDFParams &bRec) const = 0;

    /**
     * \brief Return the type of object (i.e. Mesh/BSDF/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EBSDF; }

    /**
     * \brief Return whether or not this BRDF is diffuse. This
     * is primarily used by photon mapping to decide whether
     * or not to store photons on a surface
     */
    virtual bool isDiffuse() const { return false; }
};

NORI_NAMESPACE_END
