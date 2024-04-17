#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class DirectLightingIntegrator : public Integrator {
public:
    DirectLightingIntegrator(const PropertyList &props) { /* No parameters this time */ }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.0f);
        }

        /* If directly hit light, return the radiance of light */
        if (its.mesh->isEmitter()) {
            return its.mesh->getEmitter()->eval(EmitterParams());
        }

        /* Shoot ray to sample incoming direct radiance */
        Ray3f shadow_ray;
        shadow_ray.o = its.p + Epsilon * its.shFrame.n;
        shadow_ray.d = its.toWorld(Warp::squareToUniformHemisphere(sampler->next2D()));
        shadow_ray.mint = Epsilon;
        shadow_ray.update();
        Intersection shadow_its;

        /* Return black since the ray did not reach light */
        if (!scene->rayIntersect(shadow_ray, shadow_its) || !shadow_its.mesh->isEmitter()) {
            return Color3f(0.0f);
        }

        /* Return the incoming direct radiance from this direction */
        Color3f color = shadow_its.mesh->getEmitter()->eval(EmitterParams());
        color *= (shadow_ray.d.dot(its.shFrame.n));
        color *= 2.0;
        return color;
    }

    std::string toString() const { return "DirectLightingIntegrator[]"; }
};

NORI_REGISTER_CLASS(DirectLightingIntegrator, "direct_lighting");
NORI_NAMESPACE_END
