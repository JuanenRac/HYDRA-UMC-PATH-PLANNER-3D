# Security Policy 🔒 (HYDRA-UMC-PATH-PLANNER-3D)

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 0.x.x  | ✅ Yes             |

## Reporting a Vulnerability

**CRITICAL: Do not report safety-critical vulnerabilities through public GitHub issues.**

In a multi-robot workspace, a path planning flaw can lead to physical collisions. If you discover a vulnerability affecting the **collision checking logic**, **Octree integrity**, or **trajectory scaling bypasses**:

1. **Email**: Send a detailed report to `electrohobby3d@gmail.com`.
2. **Impact**: Describe if the bug allows generating paths that collide with known obstacles, bypassing robotic joint limits, or causing swarm-wide lockups.
3. **Response**: Initial acknowledgment within 48 hours.

We follow a coordinated disclosure policy to ensure hardware safety before public release.
