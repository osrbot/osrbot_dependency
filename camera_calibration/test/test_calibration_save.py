from io import BytesIO
import tarfile

from camera_calibration.calibrator import Calibrator


def test_extract_yaml_files_writes_requested_path(tmp_path):
    tar_path = tmp_path / 'calibrationdata.tar.gz'
    yaml_path = tmp_path / 'camera_info' / 'rgb.yaml'

    with tarfile.open(tar_path, 'w:gz') as tf:
        data = b'image_width: 640\n'
        info = tarfile.TarInfo('ost.yaml')
        info.size = len(data)
        tf.addfile(info, BytesIO(data))

    Calibrator.extract_yaml_files(str(tar_path), str(tmp_path), str(yaml_path))

    assert yaml_path.read_text() == 'image_width: 640\n'
