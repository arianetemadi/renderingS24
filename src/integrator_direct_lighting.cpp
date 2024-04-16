#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class DirectLightingIntegrator : public Integrator {
 public:
  DirectLightingIntegrator(const PropertyList &props) { /* No parameters this time */
  }

  Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
    /* Find the surface that is visible in the requested direction */
    Intersection its;
    if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

    /* Return the ambient-occlusion value as a color */
    Ray3f shadow_ray;
    shadow_ray.o = its.p;
    shadow_ray.d = its.toWorld(Warp::squareToUniformHemisphere(sampler->next2D()));
    shadow_ray.mint = Epsilon;
    shadow_ray.maxt = scene->getBoundingBox().getExtents().norm();
    shadow_ray.update();
    Intersection shadow_its;
    Color3f occ;
    if (!scene->rayIntersect(shadow_ray, shadow_its)) {
        occ = {1.0, 1.0, 1.0};
    } else {
        occ = {0.0, 0.0, 0.0};
    }
    occ *= (shadow_ray.d.dot(its.shFrame.n));
    occ *= 2.0;
    return occ;
  }

  std::string toString() const { return "DirectLightingIntegrator[]"; }
};

NORI_REGISTER_CLASS(DirectLightingIntegrator, "direct_lighting");
NORI_NAMESPACE_END
