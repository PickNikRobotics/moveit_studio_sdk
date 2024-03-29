from setuptools import find_packages, setup

package_name = "moveit_studio_py"

setup(
    name=package_name,
    version="3.0.0",
    packages=find_packages(),
    zip_safe=True,
    maintainer="Ashton Larkin",
    maintainer_email="ashton.larkin@picknik.ai",
    description="Python interface for MoveIt Pro SDK",
    license="Proprietary",
)
