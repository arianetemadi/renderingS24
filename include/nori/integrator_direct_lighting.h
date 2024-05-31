#include <utility>

#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DirectLightingIntegrator : public Integrator {
public:
    DirectLightingIntegrator(bool surfaceSampling) : surfaceSampling(surfaceSampling) { }

    DirectLightingIntegrator(const PropertyList &props) {
        surfaceSampling = props.getBoolean("surface_sampling", false);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const;

    Color3f Li_one_bounce(const Scene *scene, Sampler *sampler, const Ray3f &ray,
            Intersection &its) const;

    std::string toString() const {
        return tfm::format("DirectLightingIntegrator[surfaceSampling=%i]", surfaceSampling);
    }


private:
    bool surfaceSampling;    
};

NORI_NAMESPACE_END
