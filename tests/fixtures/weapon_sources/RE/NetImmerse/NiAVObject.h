#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace RE {
    struct NiPoint3 { float x{},y{},z{}; };
    struct NiMatrix3 { float entry[3][3]{{1,0,0},{0,1,0},{0,0,1}}; };
    struct NiTransform { NiMatrix3 rotate{}; NiPoint3 translate{}; float scale{1}; };
    class NiNode;
    class NiAVObject: public std::enable_shared_from_this<NiAVObject> {
    public:
        NiTransform local{},world{};
        std::string name;
        NiNode* parent{};
        virtual ~NiAVObject()=default;
        virtual NiNode* IsNode(){return nullptr;}
    };
    // Model reference retention; scene layout and game allocation are not tested.
    template<class T> class NiPointer {
        std::shared_ptr<T> value;
    public:
        void reset(T* node) {value=node?std::static_pointer_cast<T>(node->shared_from_this()):nullptr;}
        T* get()const {return value.get();}
        T* operator->()const {return get();}
    };
    class NiNode: public NiAVObject {
    public:
        struct Data {std::vector<std::shared_ptr<NiAVObject>> children;} data;
        Data& GetRuntimeData(){return data;}
        NiNode* IsNode()override{return this;}
    };
    class TESObjectREFR {public: std::uint32_t GetFormID()const{return 123;}};
}
