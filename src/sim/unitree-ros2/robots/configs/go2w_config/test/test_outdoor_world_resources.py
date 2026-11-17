from pathlib import Path
import re
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
MODEL_DIR = PACKAGE_DIR / 'models'
WORLD = ET.parse(PACKAGE_DIR / 'worlds' / 'outdoor.world')
MODEL_RESOURCES = sorted({
    uri.text[len('model://'):]
    for uri in WORLD.iter('uri')
    if uri.text and uri.text.startswith('model://')
})
MODEL_NAMES = sorted({resource.split('/')[0] for resource in MODEL_RESOURCES})


def test_outdoor_resources_are_bundled():
    missing = [resource for resource in MODEL_RESOURCES
               if not (MODEL_DIR / resource).exists()]
    assert not missing, f'Missing outdoor resources: {missing}'


def test_model_descriptors_are_complete():
    missing = []
    for name in MODEL_NAMES:
        model_dir = MODEL_DIR / name
        config = model_dir / 'model.config'
        if not config.is_file():
            missing.append(str(config.relative_to(PACKAGE_DIR)))
            continue
        sdf_files = ET.parse(config).findall('sdf')
        assert sdf_files, f'No SDF declared in {config}'
        for sdf in sdf_files:
            if not (model_dir / sdf.text).is_file():
                missing.append(str((model_dir / sdf.text).relative_to(PACKAGE_DIR)))
    assert not missing, f'Missing model descriptors: {missing}'


def test_gazebo_exports_bundled_models():
    manifest = ET.parse(PACKAGE_DIR / 'package.xml')
    paths = [export.get('gazebo_model_path')
             for export in manifest.findall('./export/gazebo_ros')]
    assert '${prefix}/models' in paths


def test_models_are_installed():
    cmake = (PACKAGE_DIR / 'CMakeLists.txt').read_text()
    directories = re.findall(
        r'install\s*\(\s*DIRECTORY\s+(.*?)\s+DESTINATION\s+share/\$\{PROJECT_NAME\}',
        cmake, re.DOTALL)
    assert any('models' in group.split() for group in directories)
