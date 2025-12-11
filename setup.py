from setuptools import setup, find_packages

setup(
    name="pyrobo",
    version="0.1.0",
    description="Modular Python library for 3-DOF RRR manipulator simulation",
    author="Ganesh Arivoli",
    packages=find_packages(where="."),
    package_dir={"": "."},
    install_requires=[
        "numpy>=1.24",
        "matplotlib>=3.7",
    ],
    python_requires=">=3.8",
)

