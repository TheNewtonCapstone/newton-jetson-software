# Jetson Platform Compatibility Guide
```Node
Initial Draft: 2022-10-01, to be edited
```
## Hardware Generations & CUDA Architecture Support

### Orin Family (Ada Architecture)
- **Models**: AGX Orin, Orin NX, Orin Nano
- **CUDA Compute Capability**: 8.7
- **Supported L4T Versions**: R35.x+, R36.x
- **Key Features**: Ampere architecture, supports latest AI/ML frameworks

### Xavier Family (Volta Architecture)
- **Models**: AGX Xavier, Xavier NX
- **CUDA Compute Capability**: 7.2
- **Supported L4T Versions**: R32.x, R34.x, R35.x
- **Key Features**: Volta architecture, suitable for production deployments

### TX2 Family (Pascal Architecture)
- **Models**: TX2, TX2i, TX2 NX
- **CUDA Compute Capability**: 6.2
- **Supported L4T Versions**: R32.x
- **Key Features**: Pascal architecture, power-efficient

### Nano/TX1 Family (Maxwell Architecture)
- **Models**: Jetson Nano, TX1
- **CUDA Compute Capability**: 5.3
- **Supported L4T Versions**: R32.x
- **Key Features**: Maxwell architecture, entry-level AI capabilities

## L4T Version Compatibility Matrix

### JetPack 6.x (L4T R36.x)
- **Supported Hardware**: Orin family only
- **CUDA Versions**: 
  - R36.4: CUDA 12.6
  - R36.3: CUDA 12.4
  - R36.2: CUDA 12.2
- **Base OS**: Ubuntu 22.04
- **Container Requirements**: Only compatible with JP6 containers
- **Key Notes**: Major architecture change, not backward compatible

### JetPack 5.x (L4T R35.x/R34.x)
- **Supported Hardware**: Xavier and Orin families
- **CUDA Versions**: 
  - R35.4.1: CUDA 11.4
  - R35.3.1: CUDA 11.4
  - R35.2.1: CUDA 11.4
  - R34.1.1: CUDA 11.4
- **Base OS**: Ubuntu 20.04
- **Container Requirements**: Only compatible with JP5 containers
- **Key Notes**: Introduced Orin support, maintained Xavier support

### JetPack 4.x (L4T R32.x)
- **Supported Hardware**: All Jetson platforms (Nano, TX1, TX2, Xavier)
- **CUDA Versions**:
  - R32.7.x: CUDA 10.2
  - R32.6.x: CUDA 10.2
  - R32.5.x: CUDA 10.2
- **Base OS**: Ubuntu 18.04
- **Container Requirements**: Compatible with JP4 containers
- **Key Notes**: Last version supporting older platforms

## Container Compatibility Rules

1. **Forward Compatibility**:
   - JetPack 6.x (R36.x) containers only run on JetPack 6.x systems
   - JetPack 5.x (R35.x/R34.x) containers only run on JetPack 5.x systems
   - JetPack 4.x (R32.7.x+) containers run on JetPack 4.6.x+ systems

2. **Base Container Selection**:
   - JP6 (R36.x): Uses `ubuntu:22.04` as base
   - JP5 (R35.x): Uses `nvcr.io/nvidia/l4t-jetpack:r35.x.x`
   - JP4 (R32.x): Uses `nvcr.io/nvidia/l4t-base:r32.x.x`

3. **CUDA Architecture Support**:
   - JP6: Supports compute capability 8.7 (Orin)
   - JP5: Supports compute capabilities 7.2 (Xavier) and 8.7 (Orin)
   - JP4: Supports compute capabilities 5.3 (Nano/TX1), 6.2 (TX2), and 7.2 (Xavier)

## Package Dependencies and Version Matrix

### CUDA Dependencies
- **CUDA 12.x** (JP6):
  - Required cuDNN: 8.9+
  - Required TensorRT: 8.6+
  - Python versions: 3.8, 3.10
  
- **CUDA 11.4** (JP5):
  - Required cuDNN: 8.6
  - Required TensorRT: 8.5
  - Python versions: 3.8, 3.10

- **CUDA 10.2** (JP4):
  - Required cuDNN: 8.2
  - Required TensorRT: 8.2
  - Python versions: 3.6, 3.8

### Framework Compatibility

#### TensorFlow
- JP6: TensorFlow 2.14+
- JP5: TensorFlow 2.11+
- JP4: TensorFlow 1.15.5, 2.9

#### PyTorch
- JP6: PyTorch 2.1+
- JP5: PyTorch 1.13+
- JP4: PyTorch 1.11

## Best Practices
1. **Version Checking**:
   - Always verify L4T version compatibility before deploying containers
   - Use version detection scripts to ensure proper runtime environment
   - Check CUDA compute capabilities match target hardware

2. **Container Building**:
   - Build containers for specific L4T versions
   - Include version detection scripts in container entry points
   - Set appropriate CUDA architectures during compilation

3. **Deployment**:
   - Use containers matching the host L4T version
   - Verify hardware compatibility before deployment
   - Consider power modes and thermal constraints

4. **Development**:
   - Develop against lowest common denominator for wider compatibility
   - Test on all target platforms
   - Document version requirements clearly

## Common Issues and Solutions
1. **Container Compatibility**:
   - Issue: Container fails to start
   - Solution: Verify L4T version match between host and container

2. **CUDA Version Mismatch**:
   - Issue: CUDA driver version mismatch
   - Solution: Use containers built for specific L4T version

3. **Library Dependencies**:
   - Issue: Missing or incompatible libraries
   - Solution: Use appropriate base container for L4T version

4. **Performance Issues**:
   - Issue: Poor inference performance
   - Solution: Verify CUDA architecture matches hardware capabilities

## Resources and Tools

1. **Version Detection**:
   - Use `version.py` for comprehensive version information
   - Use `version.sh` for quick L4T version checks

2. **Container Management**:
   - Use `l4t_version_compatible()` for compatibility checks
   - Reference `get_l4t_base()` for base container selection

3. **Documentation**:
   - NVIDIA L4T Documentation
   - JetPack Release Notes
   - Container Platform Release Notes