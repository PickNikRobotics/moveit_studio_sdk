from setuptools import find_packages, setup

package_name = "moveit_studio_py"

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
    maintainer="Ashton Larkin",
    maintainer_email="ashton.larkin@picknik.ai",
    description="MoveIt Studio SDK",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "start_stop_async = moveit_studio_py.start_stop_async:main",
            "start_blocking = moveit_studio_py.start_blocking:main",
        ],
    },
)
