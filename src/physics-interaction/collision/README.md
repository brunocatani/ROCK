# Collision Module

This folder owns cross-cutting collision and contact policy that is shared by hands, grabbed objects, weapons, and the core frame loop. The boundary exists so contact routing, layer policy, suppression leases, shape-cast math, and push assist can evolve without becoming owned by whichever runtime feature happened to use them first.

Stateful runtime behavior stays in focused translation units such as `CollisionSuppressionRegistry.cpp`; pure policy/math remains header-only where tests can exercise it directly.
