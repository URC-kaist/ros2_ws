from setuptools import setup

package_name = "mr2_system_status"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "psutil"],
    zip_safe=True,
    maintainer="mr2",
    maintainer_email="gmmyung@kaist.ac.kr",
    description="System status publisher using psutil.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "system_status = mr2_system_status.system_status_node:main",
        ],
    },
)
