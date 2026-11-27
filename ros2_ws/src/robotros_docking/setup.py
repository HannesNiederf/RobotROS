from setuptools import find_packages, setup

package_name = "robotros_docking"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobotROS team",
    maintainer_email="todo@example.invalid",
    description="Docking server of robotROS: EngageCart and ReleaseCart (ADR 0009).",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "docking_server = robotros_docking.docking_server:main",
        ]
    },
)
