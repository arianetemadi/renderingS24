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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
	float x = (sample(0) <= 0.5f) ? (sqrt(2 * sample(0)) - 1) : (1 - sqrt(2 * (1 - sample(0))));
	float y = (sample(1) <= 0.5f) ? (sqrt(2 * sample(1)) - 1) : (1 - sqrt(2 * (1 - sample(1))));
	return Point2f(x, y);
}

float Warp::squareToTentPdf(const Point2f &p) {
	float pdf0 = (abs(p(0)) <= 1.0f) ? (1 - abs(p(0))) : 0.0f;
	float pdf1 = (abs(p(1)) <= 1.0f) ? (1 - abs(p(1))) : 0.0f;
	return pdf0 * pdf1;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
	float theta = sample(0) * 2 * M_PI;
	float radius = sqrt(sample(1));
	float x = radius * cos(theta);
	float y = radius * sin(theta);
	return Point2f(x, y);
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
	return (p(0) * p(0) + p(1) * p(1) <= 1) ? INV_PI : 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
	float theta = acos(1 - 2 * sample(0));
	float phi = sample(1) * 2 * M_PI;
	float st = sin(theta);
	float x = st * cos(phi);
	float y = st * sin(phi);
	float z = cos(theta);
	return Vector3f(x, y, z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
	// return (v(0) * v(0) + v(1) * v(1) + v(2) * v(2) == 1.0f) ? INV_PI / 4 : 0.0f;
	return INV_PI / 4;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float cosTheta = sample.y();
    float sinTheta = std::sqrt(std::max((float) 0, 1-cosTheta*cosTheta));

    float sinPhi, cosPhi;
    sincosf(2.0f * M_PI * sample.x(), &sinPhi, &cosPhi);

    return Vector3f(cosPhi * sinTheta, sinPhi * sinTheta, cosTheta);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    /*
    Assignment 1: Complete this function.
    */
    throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
	/*
	Assignment 3: Complete this function.
	*/
    throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
	/*
	Assignment 3: Complete this function.
	*/
    throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToPhongSpecular(Point2f sample, float exponent) {
	/*
	Assignment 3 (optional): Complete this function.
	*/
    throw NoriException("Warp::squareToPhongSpecular() is not yet implemented!");
}

float Warp::squareToPhongSpecularPdf(const Vector3f &v, float exponent) {
	/*
	Assignment 3 (optional): Complete this function.
	*/
    throw NoriException("Warp::squareToPhongSpecularPdf() is not yet implemented!");
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
	/*
	Assignment 3: Complete this function.
	*/
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
	/*
	Assignment 3: Complete this function.
	*/
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
