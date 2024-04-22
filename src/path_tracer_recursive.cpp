#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracerRecursiveIntegrator : public Integrator {
public:
    PathTracerRecursiveIntegrator(const PropertyList &props) {
        russian_roulette = props.getBoolean("rr", false);

        // Default: iterative
        iterative = props.getBoolean("iterative", true);
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

    // Recursive or iterative implementation,
    // default: iterative
    bool iterative;

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
        Color3f emitted_radiance(0.f);
        if (its.mesh->isEmitter()) {
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

        /* Sample ray in random direction */
        Ray3f sample_ray;
        sample_ray.o = its.p + Epsilon * its.shFrame.n;
        sample_ray.d = its.toWorld(Warp::squareToUniformHemisphere(sampler->next2D()));
        sample_ray.mint = Epsilon;
        sample_ray.update();

        /* Return the incoming direct radiance from this direction */
        BSDFParams params {its.toLocal(sample_ray.d), -its.toLocal(ray.d)};
        return Li_recursive(scene, sampler, sample_ray, bounce_cnt + 1)
                * (sample_ray.d.dot(its.shFrame.n))
                * its.mesh->getBSDF()->eval(params)
                * 2 * M_PI
                / kill_prob
                + emitted_radiance;
    }

    Color3f Li_iterative(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f color(0.f);
        Color3f throughput(1.f);
        int bounce_cnt = 0;
        Ray3f ray_copy(ray);

        while (true) {
            if (!russian_roulette && bounce_cnt >= max_bounces) {
                break;
            }

            Intersection its;
            if (!scene->rayIntersect(ray_copy, its)) {
                break;
            }

            Color3f emitted_radiance(0.f);
            if (its.mesh->isEmitter()) {
                emitted_radiance = its.mesh->getEmitter()->eval(EmitterParams());
            }
            color += emitted_radiance * throughput;

            /* Russian Roulette */
            bool kill_cond = (russian_roulette && bounce_cnt > rr_min_bounces);
            double kill_prob = kill_cond ? rr_prob : 1;
            if (kill_cond) {
                if (sampler->next1D() >= kill_prob) {
                    break;
                }
            }

            /* Sample ray in random direction */
            Ray3f sample_ray;
            sample_ray.o = its.p + Epsilon * its.shFrame.n;
            sample_ray.d = its.toWorld(Warp::squareToUniformHemisphere(sampler->next2D()));
            sample_ray.mint = Epsilon;
            sample_ray.update();

            /* Return the incoming direct radiance from this direction */
            BSDFParams params {its.toLocal(sample_ray.d), -its.toLocal(ray_copy.d)};

            /* Update throughput */
            throughput *= (sample_ray.d.dot(its.shFrame.n))
                        * its.mesh->getBSDF()->eval(params)
                        * 2 * M_PI
                        / kill_prob;

            bounce_cnt++;
            ray_copy = Ray3f(sample_ray);
        }

        return color;
    }
};

NORI_REGISTER_CLASS(PathTracerRecursiveIntegrator, "path_tracer_recursive");
NORI_NAMESPACE_END
