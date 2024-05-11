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
#include <stack>
#include <utility>

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
        struct alignas(32) CompactNode {  // TODO: play with alignas(32)
            BoundingBox3f bbox;
            int nLeafTriangles;  // IMPORTANT: 0 for interior nodes!
            union {
                int triRange0;
                int rightChildInd;
            };
            
            std::string toString() {
                return tfm::format("CompactNode: bbox=%s, nLeafTriangles=%d, triRange0/rightChildInd=%d", bbox.toString(), nLeafTriangles, triRange0);
            }
        };

        struct Node {
            Node (int leftRange, int rightRange) {
                triRange[0] = leftRange;
                triRange[1] = rightRange;
            }
            
            void build(std::vector<int>& triangleIndices, const Mesh* mesh,
                const std::vector<BoundingBox3f>& bboxes, const std::vector<Point3f>& centroids,
                std::vector<CompactNode>& compactBVH) {  // TODO: try array
                
                // compactly save this node
                CompactNode cn;
                cn.bbox = bbox;
                
                if (nTriangles() <= 20) {  // leaf node
                    cn.nLeafTriangles = nTriangles();
                    cn.triRange0 = triRange[0];
                    compactBVH.push_back(cn);
                }
                else {  // interior node
                    isInterior = true;
                    cn.nLeafTriangles = 0;
                    
                    // choose the largest axis for splitting
                    splitAxis = bbox.getLargestAxis();

                    // sort your range of triangles into two groups, with nelements function
                    int median = triRange[0] + nTriangles() / 2;
                    std::nth_element(triangleIndices.begin() + triRange[0],
                                    triangleIndices.begin() + median, triangleIndices.begin() + triRange[1], 
                                    [&](const int& lhs, const int& rhs)
                                    {
                                        Point3f c_lhs = centroids[lhs];
                                        Point3f c_rhs = centroids[rhs];
                                        return c_lhs[splitAxis] < c_rhs[splitAxis];
                                    });

                    // recursively call on left and right groups
                    children[0] = new Node(triRange[0], median);
                    children[1] = new Node(median, triRange[1]);
                    children[0]->computeBoundingBox(triangleIndices, mesh, bboxes);
                    children[1]->computeBoundingBox(triangleIndices, mesh, bboxes);

                    compactBVH.push_back(cn);
                    int currentCNInd = compactBVH.size() - 1;

                    children[0]->build(triangleIndices, mesh, bboxes, centroids, compactBVH);   

                    int secondChildCNInd = compactBVH.size();
                    compactBVH[currentCNInd].rightChildInd = secondChildCNInd;

                    children[1]->build(triangleIndices, mesh, bboxes, centroids, compactBVH);
                }
            }

            // for traversing the BVH recursively
            int rayIntersectRecursive(const std::vector<int>& triangleIndices, const Mesh* mesh, Ray3f &ray,
                Intersection &its, bool shadowRay) const {
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
                    if (boxHit[i]) {
                        triInd[i] = children[i]->rayIntersectRecursive(triangleIndices, mesh, ray, its, shadowRay);
                        if (triInd[i] == -2) return -2;  // shadow ray
                    }
                    // recalculate the box hit for the second child
                    // in case the ray has been shortened by intersections in the first child
                    boxHit[1 - i] = children[1 - i]->bbox.rayIntersect(ray) || children[1 - i]->bbox.contains(ray.o);
                    if (boxHit[1 - i]) {
                        triInd[1 - i] = children[1 - i]->rayIntersectRecursive(triangleIndices, mesh, ray, its, shadowRay);
                        if (triInd[1 - i] == -2) return -2;  // shadow ray
                    }
                    return (triInd[1 - i] == -1) ? triInd[i] : triInd[1 - i];
                }
                else {  // node is a leaf
                    // loop over all triangles in this leaf
                    int triInd = -1;  // assume no hit
                    for (int i = triRange[0]; i < triRange[1]; i++) {
                        // test for intersection
                        if (Accel::testTriangle(mesh, triangleIndices[i], ray, shadowRay, its)) {
                            if (shadowRay) return -2;
                            triInd = triangleIndices[i];
                            // shorten the ray to this intersection
                            ray.maxt = its.t;
                        }
                    }
                    return triInd;
                }
            }

            void computeBoundingBox(std::vector<int>& triangleIndices, const Mesh* mesh,
                const std::vector<BoundingBox3f>& bboxes) {
                // start with the bounding box of the first triangle
                bbox = bboxes[triangleIndices[triRange[0]]];

                // expand by looping through the rest
                for (int i = triRange[0] + 1; i < triRange[1]; i++) {
                    bbox.expandBy(bboxes[triangleIndices[i]]);
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
            // cache bounding boxes and centroids of all triangles
            for (int i = 0; i < mesh->getTriangleCount(); i++) {
                bboxes.push_back(mesh->getBoundingBox(i));
                centroids.push_back(mesh->getCentroid(i));
            }

            // init root
            root = new Node(0, triangleIndices.size());
            root->bbox = mesh->getBoundingBox();

            // start building the tree recursively
            root->build(triangleIndices, mesh, bboxes, centroids, compactBVH);

            // // write to .obj file
            // std::string filename = "./bboxes.obj";
           
            // // if (mesh->getVertexCount() == 6 && mesh->getTriangleCount() == 8) {
                // BoundingBox3f::writeOBJ(filename, bboxes);
            // // }

            cout << "root build finished." << endl;
        }

        // for traversing the BVH iteratively (more efficient)
        int rayIntersectIterative(const std::vector<int>& triangleIndices, const Mesh* mesh, Ray3f &ray,
            Intersection &its, bool shadowRay) const {
            std::stack<std::pair<int, CompactNode>> nodes;
            // std::stack<int> nodeInds;
            // std::pair<int, CompactNode> p(x, compactBVH[0]);
            // auto ttt = std::make_pair(x, compactBVH[0]);
            // cout << "CBVH size: " << compactBVH.size() << endl;
            nodes.push(std::pair<int, CompactNode>(0, compactBVH[0]));
            // nodeInds.push(0);
            int triInd = -1;  // assume no hit
            while (!nodes.empty()) {
                // remove the first node in the stack
                int nodeInd = nodes.top().first;
                CompactNode node = nodes.top().second;
                nodes.pop();
                // nodeInds.pop();
                // cout << node.toString() << endl;

                // if ray origin is outside the box and ray does not hit the box
                if (!node.bbox.contains(ray.o) && !node.bbox.rayIntersect(ray)) {
                    continue;
                }

                // if node is interior
                if (node.nLeafTriangles == 0) {
                    CompactNode children[2];
                    int childrenInd[2] = {nodeInd + 1, node.rightChildInd};
                    children[0] = compactBVH[childrenInd[0]];
                    children[1] = compactBVH[childrenInd[1]];
                    // check if ray hits the children bounding boxes
                    float nearT[2], farT[2];
                    children[0].bbox.rayIntersect(ray, nearT[0], farT[0]) || children[0].bbox.contains(ray.o);
                    children[1].bbox.rayIntersect(ray, nearT[1], farT[1]) || children[1].bbox.contains(ray.o);

                    // recurse into children nodes, open the closer one first
                    int i = (nearT[0] < nearT[1]) ? 0 : 1;
                    nodes.push(std::pair<int, CompactNode>(childrenInd[1 - i], children[1 - i]));
                    // nodeInds.push();
                    nodes.push(std::pair<int, CompactNode>(childrenInd[i], children[i]));
                    // nodeInds.push();
                }
                else {  // node is a leaf
                    // loop over all triangles in this leaf
                    for (int i = node.triRange0; i < node.triRange0 + node.nLeafTriangles; i++) {
                        // test for intersection
                        if (Accel::testTriangle(mesh, triangleIndices[i], ray, shadowRay, its)) {
                            if (shadowRay) return -2;
                            triInd = triangleIndices[i];
                            // shorten the ray to this intersection
                            ray.maxt = its.t;
                        }
                    }
                }
            }

            return triInd;
        }

        int rayIntersection(Ray3f &ray, Intersection &its, bool shadowRay) const {
            // check if ray hits the root bounding box first
            if (!root->bbox.rayIntersect(ray) && !root->bbox.contains(ray.o)) {
                return -1;
            }
            
            // return root->rayIntersectRecursive(triangleIndices, mesh, ray, its, shadowRay);
            return rayIntersectIterative(triangleIndices, mesh, ray, its, shadowRay);
        }

		const Mesh* getMesh() const
		{
			return mesh;
		}

     private:
		const Mesh* mesh;
        Node* root;
        std::vector<int> triangleIndices;

        // cached bounding boxes of triangles
        std::vector<BoundingBox3f> bboxes;
        // cached centroids of triangles
        std::vector<Point3f> centroids;

        // compact representation of the BVH tree for faster traversal
        std::vector<CompactNode> compactBVH;
	};

    // static std::vector<BoundingBox3f> bboxes;

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
