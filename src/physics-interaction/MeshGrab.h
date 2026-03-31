#pragma once

// MeshGrab.h — CPU-side mesh geometry access for grab point computation.
//
// WHY: HIGGS computes grab points by raycasting the palm line against the
// object's visual mesh triangles, finding the closest surface point, and
// aligning the object so that point sits in the palm. This creates natural
// grip alignment — you grab a gun by the handle, a mug by the rim, etc.
//
// FO4VR OFFSET DISCOVERY:
// NiAVObject in FO4VR is 0x160 bytes (NOT 0x120 like flat FO4).
// All BSGeometry/BSTriShape members are shifted +0x40 from CommonLibF4VR headers.
// CommonLibF4VR's C++ struct member access reads WRONG offsets — must use raw
// pointer arithmetic at the corrected VR offsets.
//
// BSGeometryData (at rendererData pointer):
//   +0x08 → VertexData* → +0x08 → vertexBlock (CPU vertex buffer)
//   +0x10 → TriangleData* → +0x08 → triangles (CPU uint16 indices)
//
// Vertex stride: (vertexDesc & 0xF) * 4 bytes per vertex.
// Position offset: (vertexDesc >> 2) & 0x3C (usually 0 for BSTriShape).
// Position format: FP16 (half-float) by default, float32 if VF_FULLPREC
//   flag is set (bit 54 of vertexDesc).

#include "PhysicsLog.h"
#include "RE/Fallout.h"

#include <vector>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>

namespace frik::rock
{
	// =========================================================================
	// VR-corrected BSGeometry/BSTriShape offsets
	// CommonLibF4VR uses flat FO4 offsets (NiAVObject=0x120).
	// FO4VR has NiAVObject=0x160, shifting everything +0x40.
	// =========================================================================
	namespace VROffset
	{
		constexpr int rendererData = 0x188;  // BSGeometry::rendererData (was 0x148)
		constexpr int vertexDesc   = 0x190;  // BSGeometry::vertexDesc (was 0x150)
		constexpr int numTriangles = 0x1A0;  // BSTriShape::numTriangles (was 0x160)
		constexpr int numVertices  = 0x1A4;  // BSTriShape::numVertices (was 0x164)
		constexpr int skinInstance = 0x180;  // BSGeometry::skinInstance (was 0x140)
	}

	// =========================================================================
	// Triangle data — three vertices in world space
	// =========================================================================
	struct TriangleData
	{
		RE::NiPoint3 v0, v1, v2;

		void applyTransform(const RE::NiTransform& t) {
			// NiTransform applies: result = rotate * point * scale + translate
			auto xform = [&](const RE::NiPoint3& p) -> RE::NiPoint3 {
				RE::NiPoint3 scaled(p.x * t.scale, p.y * t.scale, p.z * t.scale);
				RE::NiPoint3 rotated;
				rotated.x = t.rotate.entry[0][0] * scaled.x + t.rotate.entry[0][1] * scaled.y + t.rotate.entry[0][2] * scaled.z;
				rotated.y = t.rotate.entry[1][0] * scaled.x + t.rotate.entry[1][1] * scaled.y + t.rotate.entry[1][2] * scaled.z;
				rotated.z = t.rotate.entry[2][0] * scaled.x + t.rotate.entry[2][1] * scaled.y + t.rotate.entry[2][2] * scaled.z;
				return RE::NiPoint3(rotated.x + t.translate.x, rotated.y + t.translate.y, rotated.z + t.translate.z);
			};
			v0 = xform(v0);
			v1 = xform(v1);
			v2 = xform(v2);
		}
	};

	// =========================================================================
	// IEEE 754 binary16 (half-float) to binary32 (float) conversion.
	//
	// WHY: BSTriShape vertex buffers store positions as FP16 by default.
	// Only when the VF_FULLPREC flag (bit 54 of vertexDesc) is set are
	// positions stored as 32-bit floats. Reading FP16 data as float*
	// produces garbage values (millions of game units off), which was the
	// root cause of the 75gu mesh-physics offset bug.
	//
	// Implementation follows the standard IEEE 754 bit-manipulation approach:
	// extract sign/exponent/mantissa from 16-bit, remap to 32-bit layout.
	// Handles zero, denormals, normals, and infinity/NaN.
	// =========================================================================
	inline float halfToFloat(std::uint16_t h)
	{
		std::uint32_t sign     = (h >> 15) & 0x1;
		std::uint32_t exponent = (h >> 10) & 0x1F;
		std::uint32_t mantissa = h & 0x3FF;

		std::uint32_t f;

		if (exponent == 0) {
			if (mantissa == 0) {
				// Signed zero
				f = sign << 31;
			} else {
				// Denormalized number — renormalize
				// Shift mantissa until the leading 1 bit is in position 10
				exponent = 0;
				while ((mantissa & 0x400) == 0) {
					mantissa <<= 1;
					exponent++;
				}
				mantissa &= 0x3FF;  // Remove the leading 1 bit
				// FP16 denormal: value = 2^(-14) * (mantissa/1024). After renormalization
				// with n shifts, effective biased exponent = 1 - n in FP16 = (114 - n) in FP32.
				f = (sign << 31) | ((114 - exponent) << 23) | (mantissa << 13);
			}
		} else if (exponent == 0x1F) {
			// Infinity or NaN — preserve mantissa pattern
			f = (sign << 31) | (0xFF << 23) | (mantissa << 13);
		} else {
			// Normalized number — rebias exponent from FP16 (bias=15) to FP32 (bias=127)
			f = (sign << 31) | ((exponent - 15 + 127) << 23) | (mantissa << 13);
		}

		float result;
		std::memcpy(&result, &f, sizeof(float));
		return result;
	}

	// =========================================================================
	// Read a vertex position from a BSTriShape vertex buffer.
	//
	// WHY: Vertex position encoding depends on the VF_FULLPREC flag in
	// vertexDesc. Full-precision vertices store 3x float32 (12 bytes).
	// Half-precision (default) stores 3x uint16 FP16 (6 bytes).
	// The position offset within each vertex stride is computed from
	// vertexDesc bits, not hardcoded to 0 (though it IS 0 for most
	// standard BSTriShape — BSDynamicTriShape can differ).
	//
	// Parameters:
	//   vertexBase  — pointer to start of this vertex within the buffer
	//   posOffset   — byte offset to position data within the vertex
	//   fullPrec    — true if VF_FULLPREC is set (32-bit floats)
	// =========================================================================
	inline RE::NiPoint3 readVertexPosition(const std::uint8_t* vertexBase,
		std::uint32_t posOffset, bool fullPrec)
	{
		const std::uint8_t* posPtr = vertexBase + posOffset;
		if (fullPrec) {
			const float* fp = reinterpret_cast<const float*>(posPtr);
			return RE::NiPoint3(fp[0], fp[1], fp[2]);
		} else {
			const std::uint16_t* hp = reinterpret_cast<const std::uint16_t*>(posPtr);
			return RE::NiPoint3(
				halfToFloat(hp[0]),
				halfToFloat(hp[1]),
				halfToFloat(hp[2]));
		}
	}

	// =========================================================================
	// Extract triangles from a single BSTriShape using VR-corrected offsets.
	// Transforms model-space vertices to world space via triShape->world.
	// Returns number of triangles added.
	//
	// Handles both full-precision (float32) and half-precision (FP16) vertex
	// positions based on the VF_FULLPREC flag (bit 54 of vertexDesc).
	// Detects BSDynamicTriShape (type byte at +0x198 == 1) and skips them
	// for now — they use a separate pDynamicData buffer that requires
	// special handling (future task).
	// =========================================================================
	inline int extractTrianglesFromTriShape(RE::BSTriShape* triShape,
		std::vector<TriangleData>& outTriangles)
	{
		auto* base = reinterpret_cast<char*>(triShape);

		// --- Detect BSDynamicTriShape ---
		// Type byte at VR offset +0x198: BSTriShape=3, BSDynamicTriShape=1.
		// Dynamic shapes use a separate pDynamicData buffer at +0x1A0 with
		// stride=16 and full-precision positions. Skip for now.
		std::uint8_t geomType = *reinterpret_cast<std::uint8_t*>(base + 0x198);
		if (geomType == 1) {
			ROCK_LOG_INFO(MeshGrab, "Skipping BSDynamicTriShape '{}' (type=1, needs special handling)",
				triShape->name.c_str() ? triShape->name.c_str() : "(null)");
			return 0;
		}

		// Read VR-corrected offsets
		auto* rendererData = *reinterpret_cast<void**>(base + VROffset::rendererData);
		if (!rendererData) return 0;

		std::uint32_t numTris = *reinterpret_cast<std::uint32_t*>(base + VROffset::numTriangles);
		std::uint16_t numVerts = *reinterpret_cast<std::uint16_t*>(base + VROffset::numVertices);
		std::uint64_t vtxDescRaw = *reinterpret_cast<std::uint64_t*>(base + VROffset::vertexDesc);
		std::uint32_t vtxStride = static_cast<std::uint32_t>(vtxDescRaw & 0xF) * 4;

		// Compute position offset and full-precision flag from vertexDesc
		std::uint32_t posOffset = static_cast<std::uint32_t>((vtxDescRaw >> 2) & 0x3C);
		bool fullPrecision = ((vtxDescRaw >> 54) & 1) != 0;

		// Minimum stride: full-precision needs at least posOffset+12 bytes,
		// half-precision needs at least posOffset+6 bytes per vertex.
		std::uint32_t minStride = fullPrecision ? (posOffset + 12) : (posOffset + 6);
		if (numTris == 0 || numVerts == 0 || vtxStride < minStride) return 0;

		// BSGeometryData: vertexData at +0x08, triangleData at +0x10
		auto* vertexDataPtr = *reinterpret_cast<void**>(reinterpret_cast<char*>(rendererData) + 0x08);
		auto* triangleDataPtr = *reinterpret_cast<void**>(reinterpret_cast<char*>(rendererData) + 0x10);
		if (!vertexDataPtr || !triangleDataPtr) return 0;

		// CPU buffers: vertexBlock at VertexData+0x08, triangles at TriangleData+0x08
		auto* verts = *reinterpret_cast<std::uint8_t**>(reinterpret_cast<char*>(vertexDataPtr) + 0x08);
		auto* tris = *reinterpret_cast<std::uint16_t**>(reinterpret_cast<char*>(triangleDataPtr) + 0x08);
		if (!verts || !tris) return 0;

		// Use triShape's own world transform — correct rotation/scale for this mesh.
		RE::NiTransform worldTransform = triShape->world;

		int added = 0;
		for (std::uint32_t i = 0; i < numTris; i++) {
			std::uint16_t i0 = tris[i * 3 + 0];
			std::uint16_t i1 = tris[i * 3 + 1];
			std::uint16_t i2 = tris[i * 3 + 2];

			// Bounds check
			if (i0 >= numVerts || i1 >= numVerts || i2 >= numVerts) continue;

			// Read vertex positions using format-aware reader
			TriangleData tri;
			tri.v0 = readVertexPosition(verts + i0 * vtxStride, posOffset, fullPrecision);
			tri.v1 = readVertexPosition(verts + i1 * vtxStride, posOffset, fullPrecision);
			tri.v2 = readVertexPosition(verts + i2 * vtxStride, posOffset, fullPrecision);

			// Transform from model space to world space
			tri.applyTransform(worldTransform);
			outTriangles.push_back(tri);
			added++;
		}
		return added;
	}

	// =========================================================================
	// Recursively traverse the NiNode tree and extract all BSTriShape triangles.
	// Triangles are in world space via each triShape->world transform.
	// =========================================================================
	inline void extractAllTriangles(RE::NiAVObject* root,
		std::vector<TriangleData>& outTriangles,
		int maxDepth = 10)
	{
		if (!root || maxDepth <= 0) return;

		// Skip culled/hidden nodes
		if (root->flags.flags & 1) return;

		// Check if this node is a BSTriShape
		auto* triShape = root->IsTriShape();
		if (triShape) {
			extractTrianglesFromTriShape(triShape, outTriangles);
			return;  // Don't recurse into geometry children
		}

		// Recurse into NiNode children
		auto* niNode = root->IsNode();
		if (niNode) {
			auto& kids = niNode->GetRuntimeData().children;
			for (std::uint32_t i = 0; i < kids.size(); i++) {
				auto* kid = kids[i].get();
				if (kid) extractAllTriangles(kid, outTriangles, maxDepth - 1);
			}
		}
	}

	// =========================================================================
	// Math helpers
	// =========================================================================
	inline float dot(const RE::NiPoint3& a, const RE::NiPoint3& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	inline RE::NiPoint3 cross(const RE::NiPoint3& a, const RE::NiPoint3& b) {
		return RE::NiPoint3(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x);
	}

	inline RE::NiPoint3 normalize(const RE::NiPoint3& v) {
		float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		if (len < 1e-8f) return RE::NiPoint3(0, 0, 0);
		return RE::NiPoint3(v.x / len, v.y / len, v.z / len);
	}

	inline RE::NiPoint3 sub(const RE::NiPoint3& a, const RE::NiPoint3& b) {
		return RE::NiPoint3(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	// =========================================================================
	// Closest point on a triangle to a line (point + direction).
	// Returns the closest point on the triangle surface.
	// Based on HIGGS math_utils.cpp GetClosestPointOnTriangleToLine.
	// =========================================================================
	inline RE::NiPoint3 closestPointOnTriangleToLine(
		const RE::NiPoint3& linePoint, const RE::NiPoint3& lineDir,
		const TriangleData& tri, float& outDistSq)
	{
		// Project linePoint onto triangle plane
		RE::NiPoint3 edge0 = sub(tri.v1, tri.v0);
		RE::NiPoint3 edge1 = sub(tri.v2, tri.v0);
		RE::NiPoint3 triNormal = normalize(cross(edge0, edge1));

		// Find closest point on line to triangle plane
		float denom = dot(triNormal, lineDir);
		float t = 0.0f;
		if (std::abs(denom) > 1e-8f) {
			t = dot(sub(tri.v0, linePoint), triNormal) / denom;
		}

		// Point on line closest to plane
		RE::NiPoint3 lineOnPlane(
			linePoint.x + lineDir.x * t,
			linePoint.y + lineDir.y * t,
			linePoint.z + lineDir.z * t);

		// Now find closest point on triangle to lineOnPlane using barycentric coords
		RE::NiPoint3 v0ToP = sub(lineOnPlane, tri.v0);
		float d00 = dot(edge0, edge0);
		float d01 = dot(edge0, edge1);
		float d11 = dot(edge1, edge1);
		float d20 = dot(v0ToP, edge0);
		float d21 = dot(v0ToP, edge1);
		// S5 FIX: Guard against degenerate triangles (zero area → division by zero).
		// Previously produced infinity which self-healed via distance comparison,
		// but explicitly skipping is more robust and avoids NaN propagation.
		float baryDenom = d00 * d11 - d01 * d01;
		if (std::abs(baryDenom) < 1e-12f) {
			// Degenerate triangle — return v0 as closest point with max distance
			outDistSq = (std::numeric_limits<float>::max)();
			return tri.v0;
		}
		float invDenom = 1.0f / baryDenom;
		float u = (d11 * d20 - d01 * d21) * invDenom;
		float v = (d00 * d21 - d01 * d20) * invDenom;

		// Clamp to triangle
		if (u >= 0.0f && v >= 0.0f && u + v <= 1.0f) {
			// Inside triangle
			outDistSq = dot(sub(lineOnPlane, linePoint), sub(lineOnPlane, linePoint));
			return lineOnPlane;
		}

		// Outside triangle — find closest point on edges
		auto closestOnSegment = [](const RE::NiPoint3& p,
			const RE::NiPoint3& a, const RE::NiPoint3& b) -> RE::NiPoint3 {
			RE::NiPoint3 ab = sub(b, a);
			float t2 = dot(sub(p, a), ab) / dot(ab, ab);
			t2 = (std::max)(0.0f, (std::min)(1.0f, t2));
			return RE::NiPoint3(a.x + ab.x * t2, a.y + ab.y * t2, a.z + ab.z * t2);
		};

		RE::NiPoint3 c0 = closestOnSegment(lineOnPlane, tri.v0, tri.v1);
		RE::NiPoint3 c1 = closestOnSegment(lineOnPlane, tri.v1, tri.v2);
		RE::NiPoint3 c2 = closestOnSegment(lineOnPlane, tri.v2, tri.v0);

		RE::NiPoint3 d0 = sub(c0, lineOnPlane);
		RE::NiPoint3 d1 = sub(c1, lineOnPlane);
		RE::NiPoint3 d2 = sub(c2, lineOnPlane);

		float dist0 = dot(d0, d0);
		float dist1 = dot(d1, d1);
		float dist2 = dot(d2, d2);

		RE::NiPoint3 closest = c0;
		float minDist = dist0;
		if (dist1 < minDist) { closest = c1; minDist = dist1; }
		if (dist2 < minDist) { closest = c2; minDist = dist2; }

		// Distance from line origin to closest point
		RE::NiPoint3 toClosest = sub(closest, linePoint);
		outDistSq = dot(toClosest, toClosest);
		return closest;
	}

	// =========================================================================
	// Find the closest point on any triangle to the palm line.
	// Uses weighted distance metric (directional + lateral) from HIGGS.
	// Only accepts front-facing triangles.
	//
	// Returns true if a valid grab point was found.
	// =========================================================================
	struct GrabPoint
	{
		RE::NiPoint3 position;   // World-space contact point on mesh surface
		RE::NiPoint3 normal;     // Surface normal at contact point
		int triangleIndex = -1;  // Index into triangle array
		float distance = 1e30f;  // Weighted distance metric
	};

	inline bool findClosestGrabPoint(
		const std::vector<TriangleData>& triangles,
		const RE::NiPoint3& palmPos,
		const RE::NiPoint3& palmDir,
		float lateralWeight,
		float directionalWeight,
		GrabPoint& outResult)
	{
		float bestDist = 1e30f;
		int bestIdx = -1;
		RE::NiPoint3 bestPos;

		for (int i = 0; i < static_cast<int>(triangles.size()); i++) {
			const auto& tri = triangles[i];

			// Triangle normal
			RE::NiPoint3 triNormal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v1)));

			// Front-face check: only accept triangles facing toward the palm
			if (dot(triNormal, palmDir) > 0.0f) continue;

			// Find closest point on this triangle to the palm line
			float distSq;
			RE::NiPoint3 closest = closestPointOnTriangleToLine(palmPos, palmDir, tri, distSq);

			// Compute weighted distance (HIGGS approach)
			RE::NiPoint3 toClosest = sub(closest, palmPos);
			float dirComponent = dot(toClosest, palmDir);
			RE::NiPoint3 dirVec(palmDir.x * dirComponent, palmDir.y * dirComponent, palmDir.z * dirComponent);
			RE::NiPoint3 latVec = sub(toClosest, dirVec);

			float dirDist = dirComponent * dirComponent;
			float latDist = dot(latVec, latVec);
			float weightedDist = directionalWeight * dirDist + lateralWeight * latDist;

			if (weightedDist < bestDist) {
				bestDist = weightedDist;
				bestIdx = i;
				bestPos = closest;
			}
		}

		if (bestIdx >= 0) {
			const auto& tri = triangles[bestIdx];
			outResult.position = bestPos;
			outResult.normal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v1)));
			outResult.triangleIndex = bestIdx;
			outResult.distance = bestDist;
			return true;
		}
		return false;
	}
}
