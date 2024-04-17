#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracerRecursiveIntegrator : public Integrator {
public:
    PathTracerRecursiveIntegrator(const PropertyList &props) { /* No parameters this time */ }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        return Li_depth(scene, sampler, ray, 0);
    }
    
    std::string toString() const { return "PathTracerRecursiveIntegrator[]"; }

private:
    Color3f Li_depth(const Scene *scene, Sampler *sampler, const Ray3f &ray, int max_bounces) const {
        /* Return black if reached the maximum number of bounces */
        if (max_bounces > 3) {
            return Color3f(0.0f);
        }

        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.0f);
        }

        /* If directly hit light, add the emitted radiance */
        Color3f emitted_radiance(0.f);
        if (its.mesh->isEmitter()) {
            emitted_radiance = its.mesh->getEmitter()->eval(EmitterParams());
        }

        /* Sample ray in random direction */
        Ray3f sample_ray;
        sample_ray.o = its.p;
        sample_ray.d = its.toWorld(Warp::squareToUniformHemisphere(sampler->next2D()));
        sample_ray.mint = Epsilon;
        sample_ray.update();

        /* Return the incoming direct radiance from this direction */
        BSDFParams params {its.toLocal(sample_ray.d), -its.toLocal(ray.d)};
        return Li_depth(scene, sampler, sample_ray, max_bounces + 1)
                * (sample_ray.d.dot(its.shFrame.n))
                * its.mesh->getBSDF()->eval(params)
                * 2 * M_PI
                + emitted_radiance;
    }
};

NORI_REGISTER_CLASS(PathTracerRecursiveIntegrator, "path_tracer_recursive");
NORI_NAMESPACE_END
