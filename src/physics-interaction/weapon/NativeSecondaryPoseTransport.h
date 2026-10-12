#pragma once

#include "physics-interaction/TransformMath.h"
#include "RE/NetImmerse/NiTransform.h"
#include <algorithm>
#include <cmath>

namespace rock::native_secondary_pose_transport
{
    struct Quality { bool finite{true}; double determinant{}, orthogonalityError{}; };
    inline Quality inspect(const RE::NiTransform& pose) noexcept
    {
        Quality out;
        out.finite=std::isfinite(pose.scale) && std::isfinite(pose.translate.x) &&
            std::isfinite(pose.translate.y) && std::isfinite(pose.translate.z);
        for (int r=0;r<3;++r) for (int c=0;c<3;++c) out.finite&=std::isfinite(pose.rotate.entry[r][c]);
        const auto& m=pose.rotate.entry;
        out.determinant=double(m[0][0])*(double(m[1][1])*m[2][2]-double(m[1][2])*m[2][1])-
            double(m[0][1])*(double(m[1][0])*m[2][2]-double(m[1][2])*m[2][0])+
            double(m[0][2])*(double(m[1][0])*m[2][1]-double(m[1][1])*m[2][0]);
        for (int r=0;r<3;++r) for (int c=r;c<3;++c) {
            double dot=0;
            for (int k=0;k<3;++k) dot+=double(m[r][k])*m[c][k];
            out.orthogonalityError=(std::max)(out.orthogonalityError,std::abs(dot-(r==c?1.0:0.0)));
        }
        return out;
    }

    // Rendered matrices have native float error and may contain authored scale.
    // The transpose shortcut is not their inverse: repeating that shortcut
    // feeds the error into every descendant on every presentation pass.
    inline bool relative(const RE::NiTransform& parent,const RE::NiTransform& world,RE::NiTransform& local) noexcept
    {
        const auto quality=inspect(parent);
        if (!quality.finite || !inspect(world).finite || std::abs(parent.scale)<1e-8f ||
            !std::isfinite(quality.determinant) || std::abs(quality.determinant)<1e-12) return false;
        const auto& m=parent.rotate.entry;
        const double a=m[0][0],b=m[0][1],c=m[0][2],d=m[1][0],e=m[1][1],f=m[1][2],g=m[2][0],h=m[2][1],i=m[2][2];
        const double inv[3][3]={{(e*i-f*h)/quality.determinant,(c*h-b*i)/quality.determinant,(b*f-c*e)/quality.determinant},
            {(f*g-d*i)/quality.determinant,(a*i-c*g)/quality.determinant,(c*d-a*f)/quality.determinant},
            {(d*h-e*g)/quality.determinant,(b*g-a*h)/quality.determinant,(a*e-b*d)/quality.determinant}};
        local.MakeIdentity();
        for (int r=0;r<3;++r) for (int column=0;column<3;++column) {
            double value=0;
            for (int k=0;k<3;++k) value+=double(world.rotate.entry[r][k])*inv[k][column];
            local.rotate.entry[r][column]=static_cast<float>(value);
        }
        const double offset[3]={double(world.translate.x)-parent.translate.x,double(world.translate.y)-parent.translate.y,double(world.translate.z)-parent.translate.z};
        float* components[3]={&local.translate.x,&local.translate.y,&local.translate.z};
        for (int axis=0;axis<3;++axis) {
            double value=0;
            for (int k=0;k<3;++k) value+=inv[k][axis]*offset[k];
            *components[axis]=static_cast<float>(value/parent.scale);
        }
        local.scale=world.scale/parent.scale;
        return inspect(local).finite;
    }

    inline bool samePose(const RE::NiTransform& a,const RE::NiTransform& b) noexcept
    {
        bool same=a.scale==b.scale && a.translate.x==b.translate.x && a.translate.y==b.translate.y && a.translate.z==b.translate.z;
        for (int r=0;r<3;++r) for (int c=0;c<3;++c) same&=a.rotate.entry[r][c]==b.rotate.entry[r][c];
        return same;
    }

    class EvaluatedPose
    {
        RE::NiTransform _local{},_publishedAnchor{},_publishedWorld{};
        bool _valid{};
    public:
        void carryPublished(const RE::NiTransform& delta) noexcept
        {
            if (!_valid) return;
            // The scene owner uses exactly this operation on the rendered
            // subtree. Keep its publication witnesses in that same frame;
            // tracked hand movement is not a new native part animation.
            _publishedAnchor=transform_math::composeTransforms(delta,_publishedAnchor);
            _publishedWorld=transform_math::composeTransforms(delta,_publishedWorld);
        }
        bool reframe(const RE::NiTransform& previous,const RE::NiTransform& target,
            const RE::NiTransform& evaluated,RE::NiTransform& result) noexcept
        {
            if (!inspect(target).finite || !inspect(evaluated).finite) return false;
            if (samePose(previous,target)) { result=evaluated; return true; }
            // Reuse the evaluated relation when these are exactly our last
            // writes. Reading our rounded world back as new animation loses
            // sub-float position information at distant cell coordinates.
            // Any external bone/geometry write captures a new evaluated pose.
            if (!_valid || !samePose(previous,_publishedAnchor) || !samePose(evaluated,_publishedWorld)) {
                if (!relative(previous,evaluated,_local)) return false;
            }
            result=transform_math::composeTransforms(target,_local);
            if (!inspect(result).finite) return false;
            _publishedAnchor=target; _publishedWorld=result; _valid=true;
            return true;
        }
    };
}
