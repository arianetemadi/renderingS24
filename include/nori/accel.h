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
	class BVH {
	 public:
		BVH(Mesh* mesh) { meshes.push_back(mesh); }

        // BVH type
        enum class Type {
            Median,
            SAH,
        };

        /* Node struct for representing a node of the BVH tree */
        struct Node {
            Node (int leftRange, int rightRange) {
                triRange[0] = leftRange;
                triRange[1] = rightRange;
            }
            
            // recursively build this node
            void build(std::vector<int>& triangleIndices, BVH::Type type,
                const std::vector<BoundingBox3f>& bboxes, const std::vector<Point3f>& centroids);

            // for traversing the BVH recursively
            int rayIntersectRecursive(const std::vector<int>& triangleIndices,
                const std::vector<Mesh*>& meshes, const std::vector<int>& meshOffset, const std::vector<int>& meshMap,
                Ray3f &ray, Intersection &its, bool shadowRay) const;

            void computeBoundingBox(std::vector<int>& triangleIndices,
                const std::vector<BoundingBox3f>& bboxes);

            int nTriangles() { return triRange[1] - triRange[0]; }

            BoundingBox3f bbox;
            Node *children[2];
            int triRange[2];  // [inclusive, exclusive), range of indices in 'triangleIndices'
            bool isInterior = false;
        };

        // build the BVH (Bounding Volume Hierarchy)
        void build();

        // for traversing the BVH iteratively (more efficient)
        int rayIntersectIterative(Node* start, Ray3f &ray, Intersection &its,
            bool shadowRay) const;

        int rayIntersection(Ray3f &ray, Intersection &its, bool shadowRay) const {
            // check if ray hits the root bounding box first
            if (!root->bbox.contains(ray.o) && !root->bbox.rayIntersect(ray)) {
                return -1;
            }

            // I implemented two versions: recursive and iterative
            // surprisingly, the recursive version is slightly faster
            return root->rayIntersectRecursive(triangleIndices, meshes, meshOffset, meshMap, ray, its, shadowRay);
            // return rayIntersectIterative(root, ray, its, shadowRay);
        }

		Mesh* getMesh() { return meshes[0]; }

        void addMesh(Mesh* mesh) { meshes.push_back(mesh); }

     private:
        std::vector<Mesh*> meshes;  // has only one mesh in case there are less than 'mergeThreshold' meshes in the scene
        std::vector<int> meshOffset;
        std::vector<int> meshMap;
        BVH::Type type = Type::SAH;  // Surface Area Heuristic by default
        Node* root;  // root of the BVH hierarchy
        std::vector<int> triangleIndices;  // each BVH *leaf* node points to a range of this list
        std::vector<BoundingBox3f> bboxes;  // cached bounding boxes of triangles
        std::vector<Point3f> centroids;  // cached centroids of triangles
	};

    class KDTree {
     public:
        KDTree(Mesh* mesh) : mesh(mesh) {}

        struct Node {
            Node(std::vector<int>& triIndices) : triIndices(triIndices) {

            }

            void build(Mesh* mesh, std::vector<BoundingBox3f>& bboxes) {
                bboxes.push_back(bbox);
                if (triIndices.size() <= 20) {  // leaf node
                    return;
                }

                isInterior = true;

                // choose the axis for splitting
                splitAxis = bbox.getLargestAxis();

                // sort this node's triangle indices into two groups for the children
                // via 'nth_element', since 'sort' is unnecessary here
                std::nth_element(triIndices.begin(),
                                triIndices.begin() + triIndices.size() / 2,
                                triIndices.end(),
                                [&](const int& lhs, const int& rhs)
                                {
                                    Point3f c_lhs = mesh->getCentroid(lhs);
                                    Point3f c_rhs = mesh->getCentroid(rhs);
                                    return c_lhs[splitAxis] < c_rhs[splitAxis];
                                });
                splitDistance = mesh->getCentroid(triIndices[triIndices.size() / 2])[splitAxis];
                // TODO: what if this is not inside the box?

                // group the triangles for each child
                std::vector<int> triIndices0;
                std::vector<int> triIndices1;
                for (int i = 0; i < triIndices.size(); i++) {
                    int g = groupTri(mesh->getBoundingBox(triIndices[i]));
                    if (g == -1 || g == 0) {
                        triIndices0.push_back(triIndices[i]);
                    }
                    if (g == 1 || g == 0) {
                        triIndices1.push_back(triIndices[i]);
                    }
                }

                // if either of the two children contain all the triangles,
                // make this node a leaf node and create no children
                if (triIndices0.size() == triIndices.size()
                        || triIndices1.size() == triIndices.size()) {
                    isInterior = false;
                    return;
                }

                // create children nodes
                children[0] = new Node(triIndices0);
                children[1] = new Node(triIndices1);
                BoundingBox3f bbox0 = bbox, bbox1 = bbox;
                bbox0.max[splitAxis] = splitDistance;
                bbox1.min[splitAxis] = splitDistance;
                children[0]->bbox = bbox0;
                children[1]->bbox = bbox1;

                // recursively call on left and right children
                children[0]->build(mesh, bboxes);
                children[1]->build(mesh, bboxes);
            }

            int rayIntersect() {

            }

            int groupTri(BoundingBox3f triBBox) {
                if (triBBox.min[splitAxis] < splitDistance
                    && triBBox.max[splitAxis] < splitDistance) {
                    return -1;
                }
                if (triBBox.min[splitAxis] > splitDistance
                    && triBBox.max[splitAxis] > splitDistance) {
                    return 1;
                }
                return 0;
            }

            std::vector<int> triIndices;
            BoundingBox3f bbox;
            bool isInterior = false;
            int splitAxis;
            float splitDistance;
            Node *children[2];
        };

        void build() {
            cout << "root build begins..." << endl;
            // create the root node
            std::vector<int> triangleIndices(mesh->getTriangleCount());
            std::iota(std::begin(triangleIndices), std::end(triangleIndices), 0);
            root = new Node(triangleIndices);
            root->bbox = mesh->getBoundingBox();

            // build the tree recursively, starting from the root
            root->build(mesh, bboxes);

            std::string filename = "./bboxes1.obj";
            if (mesh->getTriangleCount() == 2000)
                BoundingBox3f::writeOBJ(filename, bboxes);
        }

        void rayIntersect() {

        }

     private:
        Mesh* mesh;
        Node* root;
        std::vector<BoundingBox3f> bboxes;
    };

    std::vector<BVH> m_bvhs;
    std::vector<KDTree> m_kdtrees;

    BoundingBox3f m_bbox;           ///< Bounding box of the entire scene

    int mergeThreshold = 20;

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
