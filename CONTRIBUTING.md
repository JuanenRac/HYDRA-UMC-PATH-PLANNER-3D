# Contributing to HYDRA-UMC-PATH-PLANNER-3D 🦾

We welcome contributions to the 3D path optimization engine of the HYDRA-UMC platform.

## Technology Stack
- **Languages**: C++20 (Core), Rust (Collision Engine).
- **Algorithms**: RRT*, PRM, Artificial Potential Fields.
- **Data Structures**: Octrees (OctoMap), K-D Trees.
- **Physics/Kinematics**: URDF parsing, S-Curve profiling.

## Guidelines
1. **Performance First**: All path planning algorithms must be highly parallelized to keep generation time under 50ms.
2. **Collision Safety**: Any new collision checking logic must be verified against the high-fidelity `HYDRA-UMC-TWIN` simulator.
3. **URDF Support**: Ensure that new features support standard URDF joint and link definitions.
4. **Benchmarks**: Include performance benchmarks (latency and memory usage) when submitting algorithm improvements.
