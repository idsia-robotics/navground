import importlib.util
import pathlib
import shutil

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


class PrebuiltPackage(Extension):

    def __init__(self, name):
        super().__init__(name, sources=[])


class ng_build_ext(build_ext):

    def run(self):
        for ext in self.extensions:
            if isinstance(ext, PrebuiltPackage):
                self.copy_package(ext)

    def copy_package(self, ext):
        spec = importlib.util.find_spec(ext.name)
        namespace, name = ext.name.split('.')
        directory = pathlib.Path(spec.origin).parent
        extdir = pathlib.Path(self.get_ext_fullpath(ext.name)).parent
        shutil.copytree(directory, extdir / name)


setup(
    ext_modules=[PrebuiltPackage('navground.core'), PrebuiltPackage('navground.sim')],
    cmdclass={'build_ext': ng_build_ext}
)
