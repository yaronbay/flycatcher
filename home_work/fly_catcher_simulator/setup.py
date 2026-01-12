
from setuptools import setup

package_name = "fly_catcher_simulator"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sim.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@todo.todo",
    description="Simple fly/drone simulator for fly-catching homework",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fly = fly_catcher_simulator.fly:main",
            "drone = fly_catcher_simulator.drone:main",
        ],
    },
)
