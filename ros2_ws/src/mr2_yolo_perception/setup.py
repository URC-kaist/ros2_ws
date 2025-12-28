from setuptools import find_packages, setup
from glob import glob

package_name = "mr2_yolo_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mr2",
    maintainer_email="gmmyung@kaist.ac.kr",
    description="YOLO detection on RGB images with depth-based 3D localization.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_rgbd_detector = mr2_yolo_perception.yolo_rgbd_detector:main",
        ],
    },
)
