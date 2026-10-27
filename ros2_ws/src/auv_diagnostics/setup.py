from glob import glob

from setuptools import find_packages, setup

setup(
    name="auv_diagnostics",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/auv_diagnostics"]),
        ("share/auv_diagnostics", ["package.xml"]),
        ("share/auv_diagnostics/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="AUV project team",
    maintainer_email="maintainers@example.invalid",
    description="ROS2 telemetry monitoring and simulation checks for underwater vehicles.",
    license="Proprietary",
    entry_points={"console_scripts": [
        "synthetic_source = auv_diagnostics.nodes:source_main",
        "telemetry_adapter = auv_diagnostics.nodes:adapter_main",
        "diagnostic_stub = auv_diagnostics.nodes:stub_main",
        "report_recorder = auv_diagnostics.nodes:recorder_main",
        "auv_demo = auv_diagnostics.demo:main",
        "auv_monitor = auv_diagnostics.monitoring:main",
    ]},
)
