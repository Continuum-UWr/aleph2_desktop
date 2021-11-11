from setuptools import setup
from glob import glob

package_name = "aleph2_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "rqt_plugins.xml"]),
        ("share/" + package_name + "/ui", glob("resource/ui/*.ui")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Continuum Rover Team",
    maintainer_email="continuum@cs.uni.wroc.pl",
    description="Graphic interfaces for Aleph2",
    license="MIT",
    scripts=["scripts/aleph2_drivetrain_controller", "scripts/aleph2_manip_controller"],
)
