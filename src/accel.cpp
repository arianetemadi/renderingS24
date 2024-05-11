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
	for (BVH& bvh : m_bvhs)
	{
		bvh.build();
	} 
	auto after = std::chrono::system_clock::now();

	std::cout << "BVH building took " << std::chrono::duration<double>(after - before).count() << " s" << std::endl;
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;      // Was an intersection found so far?
    uint32_t closestIdx = (uint32_t) -1; // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

	for (const BVH& bvh : m_bvhs)
	{
        int triInd = bvh.rayIntersection(ray, its, shadowRay);
        if (triInd == -2) return true;  // -2 -> shadow ray
        if (triInd != -1) {
            foundIntersection = true;
            closestIdx = triInd;
        }
	}

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

void Accel::BVH::Node::build(std::vector<int>& triangleIndices, BVH::Type type,
            const std::vector<BoundingBox3f>& bboxes, const std::vector<Point3f>& centroids) {
    if (nTriangles() <= 10) {  // leaf node
        return;
    }
    
    isInterior = true;
    int splitInd;
    int splitAxis;  // 0, 1, 2 for x, y, z

    /* Median Heuristic */
    if (type == BVH::Type::Median) {    
        // choose the largest axis for splitting
        splitAxis = bbox.getLargestAxis();

        // choose the median of the range for splitting
        splitInd = nTriangles() / 2;
    }
    /* Surface Area Heuristic (SAH) */
    else if (type == BVH::Type::SAH) {
        // start at cost=infinity
        float leastCost = std::numeric_limits<float>::infinity();

        // loop over all three split axis dimensions
        for (int sa = 0; sa < 3; sa++) {
            std::sort(triangleIndices.begin() + triRange[0],
                        triangleIndices.begin() + triRange[1],
                        [&](const int& lhs, const int& rhs)
                        {
                            Point3f c_lhs = centroids[lhs];
                            Point3f c_rhs = centroids[rhs];
                            return c_lhs[sa] < c_rhs[sa];
                        });
            
            // compute the bounding boxes of left and right for all splits
            std::vector<BoundingBox3f> BBL(nTriangles() - 1);
            std::vector<BoundingBox3f> BBR(nTriangles() - 1);
            BBL[0] = bboxes[triangleIndices[triRange[0]]];
            BBR[0] = bboxes[triangleIndices[triRange[1] - 1]];
            for (int i = 1; i < nTriangles() - 1; i++) {
                BBL[i] = BoundingBox3f::merge(BBL[i - 1], bboxes[triangleIndices[triRange[0] + i]]);
                BBR[i] = BoundingBox3f::merge(BBR[i - 1], bboxes[triangleIndices[triRange[1] - 1 - i]]);
            }

            // find the split with the minimum expected cost
            for (int i = 0; i < nTriangles() - 1; i++) {
                float cost = BBL[i].getSurfaceArea() * (i + 1)
                            + BBR[nTriangles() - 2 - i].getSurfaceArea() * (nTriangles() - 1 - i);
                if (cost < leastCost) {
                    leastCost = cost;
                    splitAxis = sa;
                    splitInd = i + 1;
                }
            }
        }
    }

    // sort this node's triangle indices into two groups for the children
    // via 'nth_element', since 'sort' is unnecessary here
    std::nth_element(triangleIndices.begin() + triRange[0],
                        triangleIndices.begin() + triRange[0] + splitInd,
                        triangleIndices.begin() + triRange[1], 
                        [&](const int& lhs, const int& rhs)
                        {
                            Point3f c_lhs = centroids[lhs];
                            Point3f c_rhs = centroids[rhs];
                            return c_lhs[splitAxis] < c_rhs[splitAxis];
                        });

    // create children nodes
    children[0] = new Node(triRange[0], triRange[0] + splitInd);
    children[1] = new Node(triRange[0] + splitInd, triRange[1]);
    children[0]->computeBoundingBox(triangleIndices, bboxes);
    children[1]->computeBoundingBox(triangleIndices, bboxes);

    // recursively call on left and right children
    children[0]->build(triangleIndices, type, bboxes, centroids);                    
    children[1]->build(triangleIndices, type, bboxes, centroids);
}

int Accel::BVH::Node::rayIntersectRecursive(const std::vector<int>& triangleIndices,
            const Mesh* mesh, Ray3f &ray, Intersection &its, bool shadowRay) const {
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
        boxHit[1 - i] = children[1 - i]->bbox.contains(ray.o) || children[1 - i]->bbox.rayIntersect(ray);
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

void Accel::BVH::Node::computeBoundingBox(std::vector<int>& triangleIndices,
            const std::vector<BoundingBox3f>& bboxes) {
    // start with the bounding box of the first triangle
    bbox = bboxes[triangleIndices[triRange[0]]];

    // expand by looping through the rest
    for (int i = triRange[0] + 1; i < triRange[1]; i++) {
        bbox.expandBy(bboxes[triangleIndices[i]]);
    }
}

void Accel::BVH::build() {
    // initialize the triangle index vector
    triangleIndices = std::vector<int>(mesh->getTriangleCount());
    std::iota(std::begin(triangleIndices), std::end(triangleIndices), 0);

    // cache bounding boxes and centroids of all triangles
    bboxes.reserve(mesh->getTriangleCount());
    centroids.reserve(mesh->getTriangleCount());
    for (uint32_t i = 0; i < mesh->getTriangleCount(); i++) {
        bboxes.push_back(mesh->getBoundingBox(i));
        centroids.push_back(mesh->getCentroid(i));
    }

    // init root
    root = new Node(0, triangleIndices.size());
    root->bbox = mesh->getBoundingBox();

    // start building the tree recursively
    root->build(triangleIndices, type, bboxes, centroids);
}

int Accel::BVH::rayIntersectIterative(Node* start, Ray3f &ray, Intersection &its,
            bool shadowRay) const {
    std::stack<Node*> nodes;
    nodes.push(start);
    int triInd = -1;  // assume no hit
    while (!nodes.empty()) {
        // remove the first node in the stack
        Node* node = nodes.top();
        nodes.pop();

        // if ray origin is outside the box and ray does not hit the box
        if (!node->bbox.contains(ray.o) && !node->bbox.rayIntersect(ray)) {
            continue;
        }

        // if node is interior
        if (node->isInterior) {
            Node *children[2];
            children[0] = node->children[0];
            children[1] = node->children[1];
            // check if ray hits the children bounding boxes
            float nearT[2], farT[2];
            children[0]->bbox.rayIntersect(ray, nearT[0], farT[0]);
            children[1]->bbox.rayIntersect(ray, nearT[1], farT[1]);

            // recurse into children nodes, open the closer one first
            int i = (nearT[0] < nearT[1]) ? 0 : 1;
            nodes.push(children[1 - i]);
            nodes.push(children[i]);
        }
        else {  // node is a leaf
            // loop over all triangles in this leaf
            for (int i = node->triRange[0]; i < node->triRange[1]; i++) {
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

