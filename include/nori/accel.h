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

#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Acceleration data structure for ray intersection queries
 *
 * The current implementation falls back to a brute force loop
 * through the geometry.
 */
class Accel {
public:
    /**
     * \brief Register a triangle mesh for inclusion in the acceleration
     * data structure
     *
     * This function can only be used before \ref build() is called
     */
    void addMesh(Mesh *mesh);

    /// Build the acceleration data structure (currently a no-op)
    void build();

    /// Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene and
     * return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum extent
     *    information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \param shadowRay
     *    \c true if this is a shadow ray query, i.e. a query that only aims to
     *    find out whether the ray is blocked or not without returning detailed
     *    intersection information.
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;

private:

	/*
	Assignment 2: Extend this class!
	*/
	class BVH
	{
	private:
		const Mesh* mesh;

	public:

		BVH(const Mesh* mesh) : mesh(mesh)
		{}

		const Mesh* getMesh() const
		{
			return mesh;
		}
	};

	std::vector<BVH> m_bvhs;

    BoundingBox3f m_bbox;           ///< Bounding box of the entire scene

private:
    // Test ray for an intersection with a triangle in a mesh
	/**
     * \brief Test ray for an intersection with a triangle in a mesh and return
     * intersection data
     *
     * \param mesh Mesh containing the triangle
     * \param idx Index of the triangle
     * \param ray Reference to the ray
     * \param shadow Boolean indicating if this ray is a shadow ray
     * \param its Reference to the intersection data (output)
     */
    static bool testTriangle(
        const Mesh*    mesh,
        const uint32_t idx,
        const Ray3f&   ray,
        const bool     shadow,
        Intersection&  its
    );

    // Helper function used by rayIntersect method
    static void computeItsProps(uint32_t const& f, Intersection& its);
};

NORI_NAMESPACE_END
