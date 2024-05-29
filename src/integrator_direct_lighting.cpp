#include <utility>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

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
            /* Sample BSDF */
            BSDFRecord bRec = 
                its.mesh->getBSDF()->sample(-its.toLocal(ray.d), sampler->next2D());

            /* Sample ray */
            Ray3f sample_ray;
            sample_ray.o = its.p + Epsilon * its.shFrame.n;
            sample_ray.d = its.toWorld(bRec.params.wo);
            sample_ray.mint = Epsilon;
            sample_ray.update();
            Intersection sample_its;

            /* Return black if the ray does not reach light */
            if (!scene->rayIntersect(sample_ray, sample_its) || !sample_its.mesh->isEmitter()) {
                return Color3f(0.0f);
            }

            /* Return the incoming direct radiance from this direction */
            return sample_its.mesh->getEmitter()->eval(EmitterParams())
                    * bRec.value;
        }
        /* Surface sampling */
        else {
            /* Choose an emitter uniformly at random */
            uint32_t emitterInd = sampler->nextInt(scene->getEmitters().size());
            const Emitter* emitter = scene->getEmitters()[emitterInd];
            const Mesh* emitterMesh = emitter->getMesh();

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
            if (scene->rayIntersect(shadow_ray)) {
                return Color3f(0.0f);
            }

            Color3f color = emitter->eval(EmitterParams());  // radiance of the emitter
            /* In theory, the following two cosines should not be negative
                at this point since there is no occlusion, although in practice
                we need to catch some negatives */
            color *= std::max(0.0f, shadow_ray.d.dot(its.shFrame.n));  // cosine
            color *= std::max(0.0f, (-shadow_ray.d).dot(sample.second));  // emitter cosine
            color /= dist.squaredNorm();  // emitter distance
            BSDFParams params {its.toLocal(shadow_ray.d), -its.toLocal(ray.d)};
            color *= its.mesh->getBSDF()->eval(params);  // BRDF
            color /= emitterMesh->pdf() / scene->getEmitters().size();  // acts like multiplying by emitter surface area * #emitters

            return color;
        }
    }

    std::string toString() const {
        return tfm::format("DirectLightingIntegrator[surfaceSampling=%i]", surfaceSampling);
    }


private:
    bool surfaceSampling;    
};

NORI_REGISTER_CLASS(DirectLightingIntegrator, "direct_lighting");
NORI_NAMESPACE_END
