#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        radiance = props.getColor("radiance");
    }
        
    EmitterRecord sample(const Point3f &ref,
                const Point3f &sample) const {}

    float pdf(const EmitterParams &lRec) const {}

    Color3f eval(const EmitterParams &lRec) const {
        return radiance;
    }

    std::string toString() const { 
        return tfm::format(
            "AreaEmitter[\n"
            "  radiance = %s\n"
            "]",
            radiance.toString()
        );
    }

private:
    Color3f radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area");
NORI_NAMESPACE_END
