#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class ParallelogramEmitter : public Emitter {
public:
    ParallelogramEmitter(const PropertyList &props) {

    }
        
    EmitterRecord sample(const Point3f &ref,
                const Point3f &sample) const {};

    float pdf(const EmitterParams &lRec) const {};

    Color3f eval(const EmitterParams &lRec) const {};
    
    std::string toString() const { return "ParallelogramEmitter[]"; }
};

NORI_REGISTER_CLASS(ParallelogramEmitter, "parallelogram_emitter");
NORI_NAMESPACE_END