#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class DirectLightingIntegrator : public Integrator {
 public:
  DirectLightingIntegrator(const PropertyList &props) { /* No parameters this time */
  }

  Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
    /* Find the surface that is visible in the requested direction */
    Intersection its;
    if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

    if (its.mesh->isEmitter()) return its.mesh->getEmitter()->eval(EmitterParams());

    /* Return the ambient-occlusion value as a color */
    Ray3f shadow_ray;
    shadow_ray.o = its.p;
    shadow_ray.d = its.toWorld(Warp::squareToUniformHemisphere(sampler->next2D()));
    shadow_ray.mint = Epsilon;
    shadow_ray.update();
    Intersection shadow_its;
    Color3f light_src;
    if (scene->rayIntersect(shadow_ray, shadow_its) && shadow_its.mesh->isEmitter()) {
        light_src = shadow_its.mesh->getEmitter()->eval(EmitterParams());
    } else {
        light_src = {0.0, 0.0, 0.0};
    }
    light_src *= (shadow_ray.d.dot(its.shFrame.n));
    light_src *= 2.0;
    return light_src;
  }

  std::string toString() const { return "DirectLightingIntegrator[]"; }
};

NORI_REGISTER_CLASS(DirectLightingIntegrator, "direct_lighting");
NORI_NAMESPACE_END
