from setuptools import find_packages, setup

package_name = "mr2_heightmap"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mr2",
    maintainer_email="gmmyung@kaist.ac.kr",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pc2_to_heightmap = mr2_heightmap.pc2_to_heightmap:main",
        ],
    },
)
