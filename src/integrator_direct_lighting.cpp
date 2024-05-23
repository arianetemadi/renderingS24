#include <utility>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class DirectLightingIntegrator : public Integrator {
public:
    DirectLightingIntegrator(const PropertyList &props) {
        surfaceSampling = props.getBoolean("surface_sampling", false);
    }

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

        /* If hemisphere sampling */
        if (!surfaceSampling) {
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
        /* Surface sampling */
        else {
            // TODO: Multiple emitters
            const Mesh* emitterMesh = scene->getEmitters()[0]->getMesh();

            /* Sample point uniformly at random on the emitter mesh */
            std::pair<Point3f, Normal3f> sample = emitterMesh->samplePosition(sampler->next3D());
            Vector3f dist = sample.first - its.p;

            /* Check for occlusion */
            Ray3f shadow_ray;
            shadow_ray.o = its.p + Epsilon * its.shFrame.n;
            shadow_ray.d = dist.normalized();
            shadow_ray.mint = Epsilon;
            // we want to find an intersection before the sample point,
            // so shorten the maxt of the ray by a small amount
            shadow_ray.maxt = dist.norm() - 100 * Epsilon;
            shadow_ray.update();

            /* Return black if occluded */
            if (scene->rayIntersect(shadow_ray)) {  // todo shadow=true
                return Color3f(0.0f);
            }

            Color3f color = scene->getEmitters()[0]->eval(EmitterParams());  // radiance of the emitter
            /* In theory, the following two cosines should not be negative
                at this point since there is no occlusion, although in practice
                we need to catch some negatives */
            color *= std::max(0.0f, shadow_ray.d.dot(its.shFrame.n));  // cosine
            color *= std::max(0.0f, (-shadow_ray.d).dot(sample.second));  // emitter cosine
            color /= dist.squaredNorm();  // emitter distance
            color *= INV_PI;  // BRDF todo
            color /= emitterMesh->pdf();  // acts like multiplying by emitter surface area

            return color;
        }
    }

    std::string toString() const { return "DirectLightingIntegrator[]"; }

private:
    bool surfaceSampling;    
};

NORI_REGISTER_CLASS(DirectLightingIntegrator, "direct_lighting");
NORI_NAMESPACE_END
