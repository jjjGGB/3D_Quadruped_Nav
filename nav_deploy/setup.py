from glob import glob
from setuptools import find_packages, setup


package_name = "nav_deploy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="11chens",
    maintainer_email="1902739745@qq.com",
    description="ROS 2 deployment of the SEA-Nav Go2 ONNX navigation policy.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "nav_policy_node = nav_deploy.nav_policy_node:main",
            "rl_sar_autostart = nav_deploy.rl_sar_autostart:main",
            "verify_onnx = nav_deploy.verify_onnx:main",
        ],
    },
)
