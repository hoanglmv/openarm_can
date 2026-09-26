import re
from pathlib import Path
from xml.etree import ElementTree


WEB_ROOT = Path(__file__).parents[1] / "sim" / "web"
PACKAGE_ROOT = WEB_ROOT / "assets" / "openarm_v1" / "openarm_description"
URDF_PATH = (
    PACKAGE_ROOT
    / "assets"
    / "robot"
    / "openarm_v1.0"
    / "urdf"
    / "example"
    / "v1.urdf"
)


def test_openarm_v1_urdf_has_dashboard_joints_and_visual_meshes():
    root = ElementTree.parse(URDF_PATH).getroot()
    joint_names = {joint.attrib["name"] for joint in root.findall("joint")}
    expected = {
        *(f"openarm_left_joint{index}" for index in range(1, 8)),
        *(f"openarm_right_joint{index}" for index in range(1, 8)),
        "openarm_left_finger_joint1",
        "openarm_right_finger_joint1",
    }
    assert expected <= joint_names

    visual_paths = []
    for visual in root.findall("./link/visual"):
        mesh = visual.find("./geometry/mesh")
        if mesh is None:
            continue
        match = re.fullmatch(
            r"package://openarm_description/(.+)",
            mesh.attrib["filename"],
        )
        assert match is not None
        visual_paths.append(match.group(1))

    assert len(visual_paths) == 21
    assert all((PACKAGE_ROOT / path).is_file() for path in visual_paths)
