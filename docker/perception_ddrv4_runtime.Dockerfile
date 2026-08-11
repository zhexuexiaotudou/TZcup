FROM tzcup/opr-c-rtmdet:v3.3.0-ops

ARG ONNXRUNTIME_GPU_VERSION=1.20.2

# DDRV4-06 needs the selected RTMDet and frozen G6 Area ONNX heads in one
# CUDA process. Remove the CPU-only transitive package explicitly so provider
# fallback can never masquerade as the formal product runtime.
RUN pip uninstall --break-system-packages -y onnxruntime onnxruntime-gpu \
    && pip install --break-system-packages --no-cache-dir \
       "onnxruntime-gpu==${ONNXRUNTIME_GPU_VERSION}" \
    && python3 -c "import onnxruntime as ort; assert 'CUDAExecutionProvider' in ort.get_available_providers(), ort.get_available_providers()"

ENV LD_LIBRARY_PATH="/usr/local/lib/python3.12/dist-packages/nvidia/cublas/lib:/usr/local/lib/python3.12/dist-packages/nvidia/curand/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nvjitlink/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cuda_runtime/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cudnn/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cusolver/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nccl/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cufft/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cuda_nvrtc/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cuda_cupti/lib:/usr/local/lib/python3.12/dist-packages/nvidia/cusparse/lib:/usr/local/lib/python3.12/dist-packages/nvidia/nvtx/lib"

LABEL org.opencontainers.image.title="TZcup DDRV4 CUDA perception runtime"
LABEL org.opencontainers.image.licenses="Apache-2.0"
