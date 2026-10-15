from setuptools import find_packages, setup

PACKAGE = "noisy_odometry"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament finds installed packages through this empty marker file.
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kevin Delgado",
    maintainer_email="58478385+KevDP@users.noreply.github.com",
    description="Turns the simulation's exact wheel odometry into odometry that drifts.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": [f"noisy_odometry = {PACKAGE}.node:main"]},
)
