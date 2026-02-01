from setuptools import find_packages, setup
from glob import glob

package_name = "mr2_spectrophotometer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.json") + glob("config/*.npz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mr2",
    maintainer_email="gmmyung@kaist.ac.kr",
    description="ROS2 wrapper for the MR2 spectrophotometer toolkit.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spectrophotometer_node = mr2_spectrophotometer.spectrophotometer_node:main",
            "web_calibrate = mr2_spectrophotometer.web_calibrate:main",
            "live_view = mr2_spectrophotometer.live_view:main",
        ],
    },
)
