import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mr2_rover_description"

config_data_files = []
for root, _, files in os.walk("config"):
    if files:
        dest = os.path.join("share", package_name, root)
        config_data_files.append(
            (dest, [os.path.join(root, f) for f in files])
        )

mesh_data_files = []
for root, _, files in os.walk("meshes"):
    if files:
        dest = os.path.join("share", package_name, root)
        mesh_data_files.append(
            (dest, [os.path.join(root, f) for f in files])
        )

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
        (f"share/{package_name}/ros2_control", glob("ros2_control/*.xacro")),
        (f"share/{package_name}/worlds", glob("worlds/*")),
        (f"share/{package_name}", ["package.xml", "model.config"]),
        *config_data_files,
        *mesh_data_files,
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mr2",
    maintainer_email="mr2@todo.todo",
    description="Robot description and ros2_control assets for the MR2 rover.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
