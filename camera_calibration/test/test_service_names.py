from camera_calibration.camera_calibrator import camera_info_service_name
from camera_calibration.camera_calibrator import camera_name_from_topic


def test_camera_info_service_name_preserves_absolute_namespace():
    assert camera_info_service_name('/rgb') == '/rgb/set_camera_info'


def test_camera_info_service_name_trims_trailing_slash():
    assert camera_info_service_name('/stereo/left/') == '/stereo/left/set_camera_info'


def test_camera_name_from_topic_uses_leaf_namespace():
    assert camera_name_from_topic('/rgb') == 'rgb'


def test_camera_name_from_topic_trims_trailing_slash():
    assert camera_name_from_topic('/camera/') == 'camera'
