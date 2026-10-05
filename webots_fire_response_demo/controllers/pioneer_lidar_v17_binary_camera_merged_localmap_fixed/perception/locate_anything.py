"""Persistent native adapter for locate-anything.cpp."""

import ctypes
import json
import os
import subprocess
import tempfile
from pathlib import Path

from .detector import Detection, DetectorUnavailable


class LocateAnythingDetector:
    """Use the repository build and keep its GGUF context resident.

    The CLI is a useful one-shot tool, but starting it for every camera frame
    reloads the multi-gigabyte model.  The same repository build also exports
    ``locate_anything.dll`` through ``la_capi.h``; using that C API preserves
    the model context between calls.
    """

    _MODES = {"hybrid": 0, "slow": 1, "fast": 2}

    def __init__(self, cli_path, model_path, prompt, threads=4,
                 timeout_sec=12.0, mode="hybrid"):
        self.cli_path = Path(cli_path).resolve()
        self.model_path = Path(model_path).resolve()
        if not self.cli_path.is_file():
            raise DetectorUnavailable(f"locate-anything executable unavailable: {self.cli_path}")
        if not self.model_path.is_file():
            raise DetectorUnavailable(f"locate-anything model unavailable: {self.model_path}")
        self.prompt = str(prompt)
        self.threads = max(1, int(threads))
        self.timeout_sec = float(timeout_sec)
        self.mode_name = str(mode).strip().lower()
        if self.mode_name not in self._MODES:
            raise DetectorUnavailable(
                f"unsupported LocateAnything mode {mode!r}; use hybrid, slow, or fast"
            )
        self._dll_dir_handle = None
        self._ctx = None
        self._dll = self._load_native_library()
        self._ctx = None
        if self._dll is not None:
            self._configure_api()
            self._ctx = self._dll.la_capi_load(
                os.fsencode(str(self.model_path)), self.threads
            )
            if not self._ctx:
                raise DetectorUnavailable(self._native_error(None))

    def _load_native_library(self):
        # The supplied CLI is build/examples/cli/Release/*.exe and the shared
        # library is build/Release/locate_anything.dll in the same repository.
        build_dir = self.cli_path.parents[3]
        dll_dir = build_dir / "Release"
        dll_path = dll_dir / "locate_anything.dll"
        if not dll_path.is_file():
            return None
        if os.name == "nt" and hasattr(os, "add_dll_directory"):
            self._dll_dir_handle = os.add_dll_directory(str(dll_dir))
        try:
            library = ctypes.CDLL(str(dll_path))
        except OSError:
            return None
        # Older prebuilt DLLs contain the engine but do not export the C API.
        # In that case the repository CLI remains a valid one-shot fallback.
        return library if hasattr(library, "la_capi_abi_version") else None

    def _configure_api(self):
        self._dll.la_capi_abi_version.restype = ctypes.c_int
        self._dll.la_capi_load.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self._dll.la_capi_load.restype = ctypes.c_void_p
        self._dll.la_capi_free.argtypes = [ctypes.c_void_p]
        self._dll.la_capi_locate_path.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int,
        ]
        self._dll.la_capi_locate_path.restype = ctypes.c_void_p
        self._dll.la_capi_free_string.argtypes = [ctypes.c_void_p]
        self._dll.la_capi_last_error.argtypes = [ctypes.c_void_p]
        self._dll.la_capi_last_error.restype = ctypes.c_char_p
        if self._dll.la_capi_abi_version() != 1:
            raise DetectorUnavailable("unsupported locate-anything C API ABI")

    def _native_error(self, ctx):
        try:
            message = self._dll.la_capi_last_error(ctx)
            if message:
                return message.decode("utf-8", errors="replace")
        except (AttributeError, OSError):
            pass
        return "locate-anything native context could not be loaded"

    @staticmethod
    def _canonical_class(label):
        lowered = str(label).strip().lower()
        if "person" in lowered or "human" in lowered or "casualty" in lowered:
            return "person"
        return lowered

    def detect(self, rgb):
        """Run one inference without reloading the model."""
        try:
            import cv2
        except ImportError as exc:
            raise DetectorUnavailable("OpenCV is required for LocateAnything input conversion") from exc
        with tempfile.TemporaryDirectory(prefix="locate-anything-") as temp_dir:
            image_path = Path(temp_dir) / "frame.png"
            output_path = Path(temp_dir) / "detections.json"
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(image_path), bgr):
                raise RuntimeError("could not write LocateAnything input image")
            if self._ctx is not None:
                result_ptr = self._dll.la_capi_locate_path(
                    self._ctx,
                    os.fsencode(str(image_path)),
                    self.prompt.encode("utf-8"),
                    self._MODES[self.mode_name],
                )
                if not result_ptr:
                    raise RuntimeError(self._native_error(self._ctx))
                try:
                    payload = json.loads(ctypes.string_at(result_ptr).decode("utf-8"))
                finally:
                    self._dll.la_capi_free_string(result_ptr)
            else:
                command = [
                    str(self.cli_path), "detect", "--model", str(self.model_path),
                    "--input", str(image_path), "--prompt", self.prompt,
                    "--output", str(output_path), "--threads", str(self.threads),
                    "--mode", self.mode_name,
                ]
                completed = subprocess.run(
                    command, check=True, capture_output=True, text=True,
                    timeout=self.timeout_sec,
                )
                raw = output_path.read_text(encoding="utf-8") if output_path.is_file() else completed.stdout
                payload = json.loads(raw)

        detections = []
        for index, item in enumerate(payload.get("detections", [])):
            box = item.get("box", item.get("bbox"))
            if not box or len(box) != 4:
                continue
            x1, y1, x2, y2 = map(float, box)
            label = self._canonical_class(item.get("label", item.get("class", "object")))
            confidence = float(item.get("confidence", item.get("score", 1.0)))
            detections.append(Detection(index, label, confidence, x1, y1, x2, y2))
        return detections

    def close(self):
        if self._ctx is not None and self._dll is not None:
            self._dll.la_capi_free(self._ctx)
            self._ctx = None
        if self._dll_dir_handle is not None:
            self._dll_dir_handle.close()
            self._dll_dir_handle = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
