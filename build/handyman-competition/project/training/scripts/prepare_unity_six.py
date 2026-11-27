"""Six-class V4 entry point. Rejects legacy five-class batch registries."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('_unity_six_converter', Path(__file__).with_name('prepare_unity_pilot.py'))
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)
converter.NAMES = converter.NAMES + ['filled_ketchup']
converter.GLOBAL_IDS = converter.GLOBAL_IDS + [9]
converter.ALLOW_DISCONNECTED = True
converter.CONVERTER_VERSION = '2.2-six-multipart-bridges'

if __name__ == '__main__':
    converter.main()
