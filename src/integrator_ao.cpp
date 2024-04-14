#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AmbientOcclusionIntegrator : public Integrator {
 public:
  AmbientOcclusionIntegrator(const PropertyList &props) { /* No parameters this time */
  }

  Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
    /* Find the surface that is visible in the requested direction */
    Intersection its;
    if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

    /* Return the ambient-occlusion value as a color */
    Point2f sample = sampler->next2D();
    Vector3f hemisphere_sample_local = Warp::squareToUniformHemisphere(sample);
    Vector3f hemisphere_sample_global = its.toWorld(hemisphere_sample_local);
    
    Ray3f shadow_ray;
    shadow_ray.o = its.p;
    shadow_ray.d = hemisphere_sample_global;
    shadow_ray.maxt = scene->getBoundingBox().getExtents().norm();
    shadow_ray.update();
    Intersection shadow_its;
    Color3f occ;
    if (!scene->rayIntersect(shadow_ray, shadow_its)) {
        occ = {1.0, 1.0, 1.0};
    } else {
        occ = {0.0, 0.0, 0.0};
    }
    occ *= (hemisphere_sample_global.dot(its.shFrame.n));
    occ *= 2;
    return occ;
  }

  std::string toString() const { return "AmbientOcclusionIntegrator[]"; }
};

NORI_REGISTER_CLASS(AmbientOcclusionIntegrator, "ao");
NORI_NAMESPACE_END
