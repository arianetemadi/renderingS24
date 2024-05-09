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
            void initInterior (int leftRange, int rightRange) {
                triRange[0] = leftRange;
                triRange[1] = rightRange;
            }
            
            void initLeaf () {

            }

            void build(std::vector<int>& triangleIndices, const Mesh* mesh, std::vector<BoundingBox3f>& bboxes) {
                bboxes.push_back(bbox);
                // cout << this << " My range: " << triRange[0] << " " << triRange[1] << endl;
                // cout << " My bbox: " << bbox.toString() << endl; 
                if (nTriangles() <= 10) {  // leaf node

                } else {  // interior node
                    isInterior = true;
                    
                    // choose the largest axis for splitting
                    splitAxis = bbox.getLargestAxis();
                    // splitAxis = 0;

                    // sort your range of triangles into two groups, with nelements function
                    int median = nTriangles() / 2;
                    std::nth_element(triangleIndices.begin(), triangleIndices.begin() + median, triangleIndices.end(), 
                    [&](const int& lhs, const int& rhs)
                    {
                        Point3f c_lhs = mesh->getCentroid(lhs);
                        Point3f c_rhs = mesh->getCentroid(rhs);
                        return c_lhs[splitAxis] < c_rhs[splitAxis];
                    });

                    // recursively call on left and right groups
                    Node left, right;
                    left.initInterior(triRange[0], triRange[0] + median);
                    right.initInterior(triRange[0] + median, triRange[1]);
                    left.computeBoundingBox(triangleIndices, mesh);
                    right.computeBoundingBox(triangleIndices, mesh);

                    children[0] = &left;
                    children[1] = &right;

                    left.build(triangleIndices, mesh, bboxes);                    
                    right.build(triangleIndices, mesh, bboxes);
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
            std::vector<BoundingBox3f> bboxes;
            // init root
            root.triRange[0] = 0;
            root.triRange[1] = triangleIndices.size();
            root.bbox = mesh->getBoundingBox();

            // root.computeBoundingBox(triangleIndices, mesh);

            // start building the tree recursively
            root.build(triangleIndices, mesh, bboxes);

            // print the whole triangleIndices vector, todo: why is not almost sorted, but not fully?
            // for (int i = 0; i < triangleIndices.size() && i < 200; i++) {
            //     auto t = triangleIndices[i];
            //     cout << mesh->getCentroid(t)[0] << endl;
            // }

            // bounding box computing sanity check:
            // cout << "Mesh BBOX: " << mesh->getBoundingBox().toString() << endl;
            // root.computeBoundingBox(triangleIndices, mesh);
            // cout << "Acc. BBOX: " << root.bbox.toString() << endl;

            // write to .obj file
            std::string filename = "./bboxes.obj";
           
            // if (mesh->getVertexCount() == 6 && mesh->getTriangleCount() == 8) {
                BoundingBox3f::writeOBJ(filename, bboxes);
            // }

            cout << "root build finished." << endl;
        }

		const Mesh* getMesh() const
		{
			return mesh;
		}

     private:
		const Mesh* mesh;
        Node root;
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
