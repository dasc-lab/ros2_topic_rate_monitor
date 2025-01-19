from setuptools import find_packages, setup

package_name = "topic_rate_monitor"

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
    maintainer="devansh",
    maintainer_email="devansh@umich.edu",
    description="A small utility package to monitor if topics are being published at the right rates",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "topic_rate_monitor = topic_rate_monitor.topic_rate_monitor:main"
        ],
    },
)
