# Available at setup time due to pyproject.toml
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup
import os.path as path
import glob
import platform

__version__ = "0.0.1"


libraries = []
if platform.system() == "Windows":
    libraries = ["Ws2_32"] # htonl and ntohs


ext_modules = [
    Pybind11Extension("PyWeightMap",
        sorted(glob.glob(path.join(".", "src", "*.cpp"))),
        libraries=libraries,
        cxx_std = "14"
        ),
]

setup(
    name="PyWeightMap",
    version=__version__,
    author="William Mosier",
    author_email="willmoiser@gmail.com",
    url="https://github.com/wimos-ai/PyWeightmap",
    description="A light weight and fast occupancy grid for Python3",
    long_description="",
    ext_modules=ext_modules,
    # extras_require={"test": "pytest"}, causes errors :(
    test_suite="tests",
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)
