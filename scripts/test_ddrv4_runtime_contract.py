from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "scripts/validate_ddrv4_runtime.py").read_text(encoding="utf-8")
DOCKERFILE = (ROOT / "docker/perception_ddrv4_runtime.Dockerfile").read_text(encoding="utf-8")


def test_runtime_preflight_creates_a_real_cuda_area_session():
    assert 'ort.InferenceSession(' in SOURCE
    assert 'providers=["CUDAExecutionProvider"]' in SOURCE
    assert 'active[0] != "CUDAExecutionProvider"' in SOURCE
    assert 'torch.cuda.is_available()' in SOURCE


def test_runtime_image_pins_gpu_ort_and_nvidia_library_paths():
    assert 'onnxruntime-gpu==${ONNXRUNTIME_GPU_VERSION}' in DOCKERFILE
    assert 'ARG ONNXRUNTIME_GPU_VERSION=1.20.2' in DOCKERFILE
    assert 'dist-packages/nvidia/cudnn/lib' in DOCKERFILE
