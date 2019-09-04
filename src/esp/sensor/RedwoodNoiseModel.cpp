#include "RedwoodNoiseModel.h"

#include <cuda_runtime.h>

namespace esp {
namespace sensor {

namespace {

struct CudaDeviceContext {
  CudaDeviceContext(int deviceId) {
    cudaGetDevice(&currentDevice);
    if (deviceId != currentDevice) {
      cudaSetDevice(deviceId);
      setDevice = true;
    }
  }

  ~CudaDeviceContext() {
    if (setDevice)
      cudaSetDevice(currentDevice);
  }

 private:
  bool setDevice = false;
  int currentDevice = -1;
};

}  // namespace

RedwoodNoiseModelGPUImpl::RedwoodNoiseModelGPUImpl(
    const Eigen::Ref<const Eigen::RowMatrixXf> model,
    int gpuDeviceId)
    : gpuDeviceId_{gpuDeviceId} {
  CudaDeviceContext ctx{gpuDeviceId_};

  cudaMalloc(&devModel_, model.rows() * model.cols() * sizeof(float));
  cudaMemcpy(devModel_, model.data(),
             model.rows() * model.cols() * sizeof(float),
             cudaMemcpyHostToDevice);
  curandStates_ = impl::getCurandStates();
}

RedwoodNoiseModelGPUImpl::~RedwoodNoiseModelGPUImpl() {
  CudaDeviceContext ctx{gpuDeviceId_};

  if (devModel_ != nullptr)
    cudaFree(devModel_);
  impl::freeCurandStates(curandStates_);
}

Eigen::RowMatrixXf RedwoodNoiseModelGPUImpl::simulateFromCPU(
    const Eigen::Ref<const Eigen::RowMatrixXf> depth) {
  CudaDeviceContext ctx{gpuDeviceId_};

  Eigen::RowMatrixXf noisyDepth(depth.rows(), depth.cols());

  impl::simulateFromCPU(depth.data(), depth.rows(), depth.cols(), devModel_,
                        curandStates_, noisyDepth.data());
  return noisyDepth;
}

void RedwoodNoiseModelGPUImpl::simulateFromGPU(const float* devDepth,
                                               const int rows,
                                               const int cols,
                                               float* devNoisyDepth) {
  impl::simulateFromGPU(devDepth, rows, cols, devModel_, curandStates_,
                        devNoisyDepth);
}

}  // namespace sensor
}  // namespace esp
