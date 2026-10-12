#include "physics-interaction/weapon/NativeEquippedAnimationAudio.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiNode.h"
#include <REL/Relocation.h>
#include <string>
#include <string_view>

namespace rock::native_equipped_animation_audio
{
    namespace
    {
        template<class F> F native(std::uintptr_t rva) { return reinterpret_cast<F>(REL::Offset(rva).address()); }
        template<std::size_t N> bool matches(std::uintptr_t rva,const std::array<std::uint8_t,N>& expected)
        {
            std::array<std::uint8_t,N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()),actual.data(),N) && actual==expected;
        }
        void release(RE::BSSoundHandle& handle, bool stop)
        {
            if (handle.soundID==UINT32_MAX) return;
            if (stop) (void)native<bool(*)(RE::BSSoundHandle*)>(0x1B4AA90)(&handle);
            (void)native<bool(*)(RE::BSSoundHandle*,bool)>(0x1B4B240)(&handle,true);
        }
    }
    bool Channel::validate() noexcept
    {
        // 172DB0/173820 resolve native descriptors and build/follow/play a
        // sound; 81FCD0 is the same operation with a caller-owned handle.
        return matches(0x81FCD0,std::to_array<std::uint8_t>({0x48,0x85,0xD2,0x0F,0x84,0xAE,0,0,0})) &&
            matches(0x81FDE0,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20})) &&
            matches(0x1B4AA90,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x8B,0x19})) &&
            matches(0x1B4B240,std::to_array<std::uint8_t>({0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10})) &&
            matches(0x1B4AB70,std::to_array<std::uint8_t>({0x40,0x53,0x48,0x83,0xEC,0x20,0x8B,0x19})) &&
            matches(0x4A80F0,std::to_array<std::uint8_t>({0x48,0x83,0xEC,0x28,0x48,0x8B,0x49,0x28}));
    }
    void Channel::stop() noexcept
    {
        for (auto& voice:_voices) {
            release(voice.handle,true);
            voice={};
        }
    }
    bool Channel::event(RE::Actor* player,RE::NiAVObject* model,const RE::BSFixedString& tag,
        const RE::BSFixedString& payload,std::uint64_t frame)
    {
        const bool play=tag==RE::BSFixedString("SoundPlay"), at=tag==RE::BSFixedString("SoundPlayAt");
        const bool stop=tag==RE::BSFixedString("SoundStop"), done=tag==RE::BSFixedString("SoundRelease");
        const bool all=tag==RE::BSFixedString("SoundAnimationEnd");
        if (!play && !at && !stop && !done && !all) return false;
        if (all) {
            // Native 177610 releases all handles on animation end; it does
            // not send the immediate-stop request used for SoundStop.
            for (auto& voice:_voices) { release(voice.handle,false); voice={}; }
            return true;
        }
        if (stop || done) {
            for (auto& voice:_voices) if (voice.payload==payload) {
                release(voice.handle,stop);
                voice={};
            }
            return true;
        }
        if (!player || !model || !payload.c_str() || !payload.c_str()[0]) return true;
        std::string_view name(payload.c_str());
        RE::NiAVObject* follow=model;
        std::string descriptorName;
        if (at) {
            // Native SoundPlayAt is descriptor.node. Search only this actual
            // equipped model: a same-named bone on the other gun is unrelated.
            const auto separator=name.find('.');
            if (separator==std::string_view::npos || !separator || separator+1==name.size()) {
                ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO sound payload invalid tag='{}' payload='{}'",tag.c_str(),payload.c_str());
                return true;
            }
            descriptorName.assign(name.substr(0,separator));
            name.remove_prefix(separator+1);
            const auto end=name.find('.');
            const RE::BSFixedString bone(std::string(name.substr(0,end)).c_str());
            follow=model->GetObjectByName(bone);
            if (!follow) {
                ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO sound target absent payload='{}' model={:#x}",payload.c_str(),reinterpret_cast<std::uintptr_t>(model));
                return true;
            }
        } else descriptorName.assign(name);
        const RE::BSFixedString soundTag(descriptorName.c_str());
        // Preserve the native actor sound-tag override before the editor-ID
        // resolver, as 172EE1..173106 does for player animation notifications.
        auto* base=player->GetObjectReference();
        auto* descriptor=base?base->GetSoundForTag(soundTag):nullptr;
        if (!descriptor) descriptor=native<RE::BGSSoundDescriptorForm*(*)(const char*)>(0x81FDE0)(descriptorName.c_str());
        if (!descriptor) {
            ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO native sound descriptor absent payload='{}'",payload.c_str());
            return true;
        }
        const bool looping=native<bool(*)(const RE::BGSSoundDescriptorForm*)>(0x4A80F0)(descriptor);
        Voice* slot=nullptr;
        for (auto& voice:_voices) if (voice.payload==payload) {
            // Native 173820 keeps one active loop per payload. Replacing a
            // non-looping entry drops its handle value; the audio engine owns
            // that one-shot until completion. The native record's destruction
            // only releases its string, so no stop is sent on replacement.
            if (voice.looping && native<bool(*)(RE::BSSoundHandle*)>(0x1B4AB70)(&voice.handle)) return true;
            slot=&voice;
            break;
        }
        if (!slot) for (auto& voice:_voices) {
            if (voice.handle.soundID!=UINT32_MAX && frame>voice.frame+1 &&
                !native<bool(*)(RE::BSSoundHandle*)>(0x1B4AB70)(&voice.handle)) {
                release(voice.handle,false);
                voice={};
            }
            if (!slot && voice.handle.soundID==UINT32_MAX) slot=&voice;
        }
        if (!slot) {
            ROCK_LOG_SAMPLE_WARN(Animation,1000,"AKIMBO sound capacity exhausted payload='{}'; active voices preserved",payload.c_str());
            return true;
        }
        *slot={}; slot->payload=payload; slot->frame=frame; slot->looping=looping;
        native<void(*)(RE::BSSoundHandle*,RE::BGSSoundDescriptorForm*,std::uint32_t,const RE::NiPoint3*,RE::NiAVObject*)>(0x81FCD0)(
            &slot->handle,descriptor,0,nullptr,follow);
        ROCK_LOG_SAMPLE_INFO(Animation,250,"AKIMBO native animation sound payload='{}' descriptor={:08X} handle={} model={:#x}",
            payload.c_str(),descriptor->formID,slot->handle.soundID,reinterpret_cast<std::uintptr_t>(model));
        return true;
    }
}
