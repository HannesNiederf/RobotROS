from setuptools import find_packages, setup

package_name = "robotros_mission"

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
    description="Mission layer of robotROS. So far: activity_narrator and activity_view (ADR 0004).",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "activity_narrator = robotros_mission.activity_narrator:main",
            "activity_view = robotros_mission.activity_view:main",
        ]
    },
)
