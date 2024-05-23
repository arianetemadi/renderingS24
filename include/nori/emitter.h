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
 * for evaluation methods of the \ref Emitter
 */
struct EmitterParams {
    /// Sampled position on the light source
    Point3f  p;
    /// Associated surface normal
    Normal3f n;
    /// Direction vector from reference position to 'p'
    Vector3f d;
    /// Distance between reference position and 'p'
    float    dist;

    // Default constructor
    EmitterParams() = default;

    /**
     * \brief Create a prams struct that can be used to query the
     * sampling density after having intersected an area emitter
     */
    EmitterParams(const Point3f& ref, const Point3f& p, const Normal3f& n)
        : p(p), n(n) {
        d    = p - ref;
        dist = d.norm();
        d /= dist;
    }

    /**
     * \brief Create a params struct that can be used to query the
     * sampling density after having intersected an environment emitter
     */
    EmitterParams(const Ray3f& ray)
        : p(ray(1)), n(-ray.d), d(ray.d),
          dist(std::numeric_limits<float>::infinity()) {}

    std::string toString() const;
};

/**
 * \brief Data record for conveniently querying and sampling the
 * direct illumination technique implemented by a \ref Emitter
 */

struct EmitterRecord {
    /// Parameters necessary for evaluations with the same emitter query
    EmitterParams params;
    /// Solid angle density wrt. reference position
    float         pdf;
    /// An importance weight associated with the sample. Includes any
    /// geometric terms between the emitter and the reference point.
    Color3f       w;
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:
    /**
     * \brief Direct illumination sampling: given a reference point in the
     * scene, sample an emitter position that contributes towards it
     * or fail.
     *
     * Given an arbitrary reference point in the scene, this method
     * samples a position on the emitter that has a nonzero contribution
     * towards that point. Note that this does not yet account for visibility.
     *
     * Ideally, the implementation should importance sample the product of
     * the emission profile and the geometry term between the reference point
     * and the position on the emitter.
     *
     * \param ref
     *    Reference point in the scene
     *
     * \param sample
     *    A uniformly distributed 3D vector
     *
     * \return
     *    A lumaire query record that specifies the position sample and
     *    related information.
     */
    virtual EmitterRecord sample(const Point3f &ref,
            const Point3f &sample) const = 0;

    /**
     * \brief Compute the sampling density of the direct illumination technique
     * implemented by \ref sample() with respect to the solid angle measure
     */
    virtual float pdf(const EmitterParams &lRec) const = 0;

    /// Evaluate the emitted radiance
    virtual Color3f eval(const EmitterParams &lRec) const = 0;

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }

    void setMesh(Mesh* m) { mesh = m; hasMesh = true; }

    const Mesh* getMesh() const { return mesh; }

    void printHasMesh() {
        cout << "hasMesh: " << hasMesh << endl;
    }

private:
    Mesh* mesh;
    bool hasMesh = false;
};

inline std::string EmitterParams::toString() const {
    return tfm::format(
        "EmitterParams[\n"
        "  p = %s,\n"
        "  n = %s,\n"
        "  d = %s,\n"
        "  dist = %f\n"
        "]",
        p.toString(),
        n.toString(),
        d.toString(),
        dist
    );
}

NORI_NAMESPACE_END
