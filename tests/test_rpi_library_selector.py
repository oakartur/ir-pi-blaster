import os
import types
import unittest
from unittest import mock

import rpi_library_selector as selector


class ParsePiVersionTests(unittest.TestCase):
    def test_extracts_version_number_from_model_strings(self):
        cases = {
            "Raspberry Pi 4 Model B Rev 1.2": 4,
            "Raspberry Pi Model 3B+": 3,
            "Raspberry Pi 5": 5,
            "Raspberry Pi Compute Module 4S Rev 1.0": 4,
            "Raspberry Pi Zero W": None,
            None: None,
        }
        for model, expected in cases.items():
            with self.subTest(model=model):
                self.assertEqual(selector.parse_pi_version(model), expected)


class SelectGpioLibraryTests(unittest.TestCase):
    def setUp(self) -> None:
        patcher = mock.patch.dict(os.environ, {}, clear=True)
        self.addCleanup(patcher.stop)
        patcher.start()

    def _mock_available(self, available):
        def fake_find_spec(name):
            return types.SimpleNamespace(name=name) if name in available else None

        return mock.patch(
            "rpi_library_selector.importlib.util.find_spec",
            side_effect=fake_find_spec,
        )

    def test_prefers_lgpio_on_pi5_when_available(self):
        with self._mock_available({"lgpio"}):
            lib = selector.select_gpio_library(model="Raspberry Pi 5", version=5)
        self.assertEqual(lib, "lgpio")

    def test_falls_back_to_pigpio_when_lgpio_missing(self):
        with self._mock_available({"pigpio"}):
            lib = selector.select_gpio_library(model="Raspberry Pi 5", version=5)
        self.assertEqual(lib, "pigpio")

    def test_returns_available_library_for_older_models(self):
        with self._mock_available({"pigpio"}):
            lib = selector.select_gpio_library(model="Raspberry Pi 4", version=4)
        self.assertEqual(lib, "pigpio")

    def test_can_require_installed_libraries(self):
        with self._mock_available(set()):
            with self.assertRaises(RuntimeError):
                selector.select_gpio_library(version=5, installed_only=True)


if __name__ == "__main__":
    unittest.main()
