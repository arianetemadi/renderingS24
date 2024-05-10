#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AmbientOcclusionIntegrator : public Integrator {
public:
    AmbientOcclusionIntegrator(const PropertyList &props) { /* No parameters this time */ }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.0f);
        }

        /* Shoot ray to sample the local surrounding */
        Ray3f shadow_ray;
        shadow_ray.o = its.p + Epsilon * its.shFrame.n;
        shadow_ray.d = its.toWorld(Warp::squareToUniformHemisphere(sampler->next2D()));
        shadow_ray.mint = Epsilon;
        shadow_ray.maxt = scene->getBoundingBox().getExtents().norm();
        shadow_ray.update();

        /* Return black if hit something */
        if (scene->rayIntersect(shadow_ray)) {
            return Color3f(0.0f);
        }

        /* Return the ambient occlusion radiance */
        Color3f color(1.0f);
        color *= (shadow_ray.d.dot(its.shFrame.n));
        color *= 2.0;
        return color;
    }

    std::string toString() const { return "AmbientOcclusionIntegrator[]"; }
};

NORI_REGISTER_CLASS(AmbientOcclusionIntegrator, "ao");
NORI_NAMESPACE_END
