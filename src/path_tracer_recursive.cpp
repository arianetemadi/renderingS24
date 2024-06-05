#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/integrator_direct_lighting.h>

NORI_NAMESPACE_BEGIN

class PathTracerRecursiveIntegrator : public Integrator {
public:
    PathTracerRecursiveIntegrator(const PropertyList &props) {
        russian_roulette = props.getBoolean("rr", false);

        // The default is iterative, since it is more efficient
        iterative = props.getBoolean("iterative", true);
        
        nee = props.getBoolean("nee", false);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        if (iterative) {  // Default
            return Li_iterative(scene, sampler, ray);
        } else {
            return Li_recursive(scene, sampler, ray, 0);
        }
    }
    
    std::string toString() const { return "PathTracerRecursiveIntegrator[]"; }

private:
    // Maximum number of the bonces for the case with no RR
    int max_bounces = 3;

    // Russian Roulette (RR) parameters
    bool russian_roulette;
    int rr_min_bounces = 4;
    double rr_prob = 0.7;

    // Recursive or iterative implementation
    bool iterative;

    // Next Event Estimation (NEE)
    bool nee;

    /* Recursive version.
       Can be selected by setting the corresponding parameter in the
       scene file. */
    Color3f Li_recursive(const Scene *scene, Sampler *sampler, const Ray3f &ray, int bounce_cnt) const {
        /* Return black if reached the maximum number of bounces, only when no RR */
        if (!russian_roulette && bounce_cnt >= max_bounces) {
            return Color3f(0.0f);
        }

        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return Color3f(0.0f);
        }

        /* If directly hit light, add the emitted radiance */
        Color3f emitted_radiance(0.0f);
        if (its.mesh->isEmitter() && !(nee && bounce_cnt > 0)) {
            emitted_radiance = its.mesh->getEmitter()->eval(EmitterParams());
        }

        /* Russian Roulette */
        bool kill_cond = (russian_roulette && bounce_cnt > rr_min_bounces);
        double kill_prob = kill_cond ? rr_prob : 1;
        if (kill_cond) {
            if (sampler->next1D() >= kill_prob) {
                return emitted_radiance;
            }
        }

        /* Sample BSDF */
        BSDFRecord bRec = 
            its.mesh->getBSDF()->sample(-its.toLocal(ray.d), sampler->next2D());

        /* Sample ray in random direction */
        Ray3f sample_ray;
        sample_ray.o = its.p + Epsilon * its.shFrame.n;
        sample_ray.d = its.toWorld(bRec.params.wo);
        sample_ray.mint = Epsilon;
        sample_ray.update();

        if (nee) {
            /* Next Event Estimation (NEE) */
            Color3f direct = DirectLightingIntegrator(true).Li_one_bounce(scene, sampler, ray, its);
            Color3f indirect = Li_recursive(scene, sampler, sample_ray, bounce_cnt + 1)
                    * bRec.value;
            return (direct + indirect)
                    / kill_prob
                    + emitted_radiance;
        } else {
            /* Without NEE */
            return Li_recursive(scene, sampler, sample_ray, bounce_cnt + 1)
                    * bRec.value
                    / kill_prob
                    + emitted_radiance;
        }
    }

    /* Iterative version.
       This is default.
       It is more efficient compared to recursive. */
    Color3f Li_iterative(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f color(0.f);
        Color3f throughput(1.f);
        int bounce_cnt = 0;
        Ray3f ray_copy(ray);

        while (true) {
            /* stop if reached the maximum number of bounces, only when no RR */
            if (!russian_roulette && bounce_cnt >= max_bounces) {
                break;
            }

            /* Find the surface that is visible in the requested direction */
            Intersection its;
            if (!scene->rayIntersect(ray_copy, its)) {
                break;
            }

            /* Add the emitted radiance */
            Color3f emitted_radiance(0.f);
            if (its.mesh->isEmitter() && !(nee && bounce_cnt > 0)) {
                emitted_radiance = its.mesh->getEmitter()->eval(EmitterParams());
            }

            /* Russian Roulette */
            bool kill_cond = (russian_roulette && bounce_cnt > rr_min_bounces);
            double kill_prob = kill_cond ? rr_prob : 1;
            if (kill_cond) {
                if (sampler->next1D() >= kill_prob) {
                    break;
                }
            }
            throughput /= kill_prob;

            if (nee) {
                /* Next Event Estimation (NEE) */
                color += DirectLightingIntegrator(true).Li_one_bounce(scene, sampler, ray_copy, its)
                        * throughput
                        + emitted_radiance;
            } else {
                /* Without NEE */
                color += emitted_radiance
                        * throughput;
            }

            /* Sample BSDF */
            BSDFRecord bRec = 
                its.mesh->getBSDF()->sample(-its.toLocal(ray_copy.d), sampler->next2D());

            /* Update throughput */
            throughput *= bRec.value;

            /* Bounce the ray */
            ray_copy.o = its.p + Epsilon * its.shFrame.n;
            ray_copy.d = its.toWorld(bRec.params.wo);
            ray_copy.mint = Epsilon;
            ray_copy.update();
            bounce_cnt++;
        }

        return color;
    }
};

NORI_REGISTER_CLASS(PathTracerRecursiveIntegrator, "path_tracer_recursive");
NORI_NAMESPACE_END
