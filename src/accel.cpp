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

#include <nori/accel.h>
#include <Eigen/Geometry>
#include <chrono>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) 
{
	m_bvhs.emplace_back(mesh);
    m_bbox.expandBy(mesh->getBoundingBox());
}

void Accel::build() 
{
	auto before = std::chrono::system_clock::now();

    // merge all BVHs into one
    for (int i = 0; i < m_bvhs.size(); i++) {
        m_bvhs[0].addMesh(m_bvhs[i].getMesh());
    }

	// for (BVH& bvh : m_bvhs)  // TODO: improve this loop
	// {
	// 	bvh.build();
	// } 

    m_bvhs[0].build();

	auto after = std::chrono::system_clock::now();

	std::cout << "BVH building took " << std::chrono::duration<double>(after - before).count() << " s" << std::endl;
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;      // Was an intersection found so far?
    uint32_t closestIdx = (uint32_t) -1; // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

	// for (const BVH& bvh : m_bvhs)  // TODO: improve this loop
	// {
		// const Mesh* mesh = bvh.getMesh();
		
		/* 
		   Assignment 2
		   Make sure to perform an efficient scenen traversal here, such that the 
		   brute-force loop is replaced by something more effective. 
		*/
        int triInd = m_bvhs[0].rayIntersection(ray, its, shadowRay);
        if (triInd == -2) return true;  // -2: shadow ray
        if (triInd != -1) {
            foundIntersection = true;
            closestIdx = triInd;
        }
        // break;


		// for (uint32_t idx = 0; idx < mesh->getTriangleCount(); ++idx)
		// {
		// 	if (testTriangle(mesh, idx, ray, shadowRay, its)) 
        //     {
        //         if (shadowRay) return true;
        //         /* This intersection is now considered as the closest one. Thus we
        //             will shorten this ray and set index of the closest intersection
        //             to the index of this triangle */
        //         foundIntersection = true;
        //         ray.maxt = its.t; // No need to check for triangles behind this one
        //         closestIdx = idx;
        //     }
		// }
	// }

    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */
        computeItsProps(closestIdx, its);
    }

    return foundIntersection;
}

bool Accel::testTriangle(
    const Mesh*    mesh,
    const uint32_t idx,
    const Ray3f&   ray,
    const bool     shadow,
    Intersection&  its
) {
    its.attempts++;
    float u, v, t;
    if (mesh->rayIntersect(idx, ray, u, v, t)) {
        /* An intersection was found! Can terminate
            immediately if this is a shadow ray query */
        if (shadow) return true;

        /* Fill intersection data */
        its.t    = t;
        its.uv   = Point2f(u, v);
        its.mesh = mesh;
        
        return true;
    }
    return false;
}

void Accel::computeItsProps(uint32_t const& f, Intersection& its) {
    /* Find the barycentric coordinates */
    Vector3f bary;
    bary << 1 - its.uv.sum(), its.uv;

    /* References to all relevant mesh buffers */
    const Mesh*     mesh = its.mesh;
    const MatrixXf& V    = mesh->getVertexPositions();
    const MatrixXf& N    = mesh->getVertexNormals();
    const MatrixXf& UV   = mesh->getVertexTexCoords();
    const MatrixXu& F    = mesh->getIndices();

    /* Vertex indices of the triangle */
    uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

    Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

    /* Compute the intersection position accurately
       using barycentric coordinates */
    its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

    /* Compute proper texture coordinates if provided by the mesh */
    if (UV.size() > 0)
        its.uv = bary.x() * UV.col(idx0) + bary.y() * UV.col(idx1) +
                 bary.z() * UV.col(idx2);

    /* Compute the geometry frame */
    its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

    if (N.size() > 0) {
        /* Compute the shading frame. Note that for simplicity, the current
           implementation doesn't attempt to provide tangents that are
           continuous across the surface. That means that this code will
           need to be modified to be able use anisotropic BRDFs, which need
           tangent continuity */

        its.shFrame = Frame((bary.x() * N.col(idx0) + bary.y() * N.col(idx1) +
                             bary.z() * N.col(idx2))
                                .normalized());
    } else {
        its.shFrame = its.geoFrame;
    }
}

NORI_NAMESPACE_END

