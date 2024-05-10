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
#include <vector>
#include <numeric>

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
	class BVH {
	 public:
		BVH(const Mesh* mesh) : mesh(mesh)
		{
            cout << "BVH instantiated!" << endl;

            // initialize the triangle index vector
            triangleIndices = std::vector<int>(mesh->getTriangleCount());
            std::iota(std::begin(triangleIndices), std::end(triangleIndices), 0);

        }

        // enum class Type {
        //     Middle,
        //     Median,
        //     SAH,
        // };

        struct Node {
            Node (int leftRange, int rightRange) {
                triRange[0] = leftRange;
                triRange[1] = rightRange;
            }
            
            void build(std::vector<int>& triangleIndices, const Mesh* mesh) {
                if (nTriangles() <= 5) {  // leaf node

                }
                else {  // interior node
                    isInterior = true;
                    
                    // choose the largest axis for splitting
                    splitAxis = bbox.getLargestAxis();

                    // sort your range of triangles into two groups, with nelements function
                    int median = triRange[0] + nTriangles() / 2;
                    std::nth_element(triangleIndices.begin() + triRange[0],
                                    triangleIndices.begin() + median, triangleIndices.begin() + triRange[1], 
                                    [&](const int& lhs, const int& rhs)
                                    {
                                        Point3f c_lhs = mesh->getCentroid(lhs);
                                        Point3f c_rhs = mesh->getCentroid(rhs);
                                        return c_lhs[splitAxis] < c_rhs[splitAxis];
                                    });

                    // recursively call on left and right groups
                    children[0] = new Node(triRange[0], median);
                    children[1] = new Node(median, triRange[1]);
                    children[0]->computeBoundingBox(triangleIndices, mesh);
                    children[1]->computeBoundingBox(triangleIndices, mesh);

                    children[0]->build(triangleIndices, mesh);                    
                    children[1]->build(triangleIndices, mesh);
                }
            }

            // for traversing the bvh recursively
            int rayIntersect(const std::vector<int>& triangleIndices, const Mesh* mesh, Ray3f &ray, Intersection &its, bool shadowRay) const {  // TODO: shadow ray?
                // check if ray hits the bounding box first
                if (!bbox.rayIntersect(ray) && !bbox.contains(ray.o)) {
                    return -1;
                }

                // if node is interior
                if (isInterior) {
                    // check if ray hits the children bounding boxes
                    float nearT[2], farT[2];
                    bool boxHit[2];
                    int triInd[2];
                    boxHit[0] = children[0]->bbox.rayIntersect(ray, nearT[0], farT[0]) || children[0]->bbox.contains(ray.o);
                    boxHit[1] = children[1]->bbox.rayIntersect(ray, nearT[1], farT[1]) || children[1]->bbox.contains(ray.o);
                    triInd[0] = -1;  // -1: no hit
                    triInd[1] = -1;  // -1: no hit

                    // recurse into children nodes, open the closer one first
                    int i = (nearT[0] < nearT[1]) ? 0 : 1;
                    // if (boxHit[i]) {  // TODO: revert this
                        triInd[i] = children[i]->rayIntersect(triangleIndices, mesh, ray, its, shadowRay);
                    // }
                    // recalculate the box hit for the second child
                    // in case the ray has been shortened by intersections in the first child
                    boxHit[1 - i] = children[1 - i]->bbox.rayIntersect(ray) || children[1 - i]->bbox.contains(ray.o);
                    // if (boxHit[1 - i]) {
                        triInd[1 - i] = children[1 - i]->rayIntersect(triangleIndices, mesh, ray, its, shadowRay);
                    // }
                    return (triInd[1 - i] == -1) ? triInd[i] : triInd[1 - i];
                }
                else {  // node is a leaf
                    // loop over all triangles in this leaf
                    int triInd = -1;  // assume no hit
                    for (int i = triRange[0]; i < triRange[1]; i++) {
                        // test for intersection
                        if (Accel::testTriangle(mesh, triangleIndices[i], ray, shadowRay, its)) {
                            triInd = triangleIndices[i];
                            // shorten the ray to this intersection
                            ray.maxt = its.t;
                        }
                    }
                    return triInd;
                }
            }

            void computeBoundingBox(std::vector<int>& triangleIndices, const Mesh* mesh) {
                // start with the bounding box of the first triangle
                bbox = mesh->getBoundingBox(triangleIndices[triRange[0]]);

                // expand by looping through the rest
                for (int i = triRange[0] + 1; i < triRange[1]; i++) {
                    bbox.expandBy(mesh->getBoundingBox(triangleIndices[i]));
                }
            }

            int nTriangles() {
                return triRange[1] - triRange[0];
            }

            BoundingBox3f bbox;
            Node *children[2];
            int splitAxis;  // 0, 1, 2 for x, y, z
            int triRange[2];  // [inclusive, exclusive), continuous range of triangles in the triangleIndices vector
            bool isInterior = false;
        };

        void build() {
            // init root
            root = new Node(0, triangleIndices.size());
            root->bbox = mesh->getBoundingBox();

            // start building the tree recursively
            root->build(triangleIndices, mesh);

            // // write to .obj file
            // std::string filename = "./bboxes.obj";
           
            // // if (mesh->getVertexCount() == 6 && mesh->getTriangleCount() == 8) {
            //     BoundingBox3f::writeOBJ(filename, bboxes);
            // // }

            cout << "root build finished." << endl;
        }

        int rayIntersection(Ray3f &ray, Intersection &its, bool shadowRay) const {
            return root->rayIntersect(triangleIndices, mesh, ray, its, shadowRay);
        }

		const Mesh* getMesh() const
		{
			return mesh;
		}

     private:
		const Mesh* mesh;
        Node* root;
        std::vector<int> triangleIndices;
        // todo: preprocess bounding boxes and cenetroids
	};

    static std::vector<BoundingBox3f> bboxes;

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
