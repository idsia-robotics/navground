import os
import pathlib

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(pathlib.Path(sourcedir).resolve())


class CMakeBuild(build_ext):

    def build_extension(self, ext):
        cwd = pathlib.Path().absolute()
        ext_fullpath = pathlib.Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()
        build_temp = pathlib.Path(self.build_temp).parent
        build_temp.mkdir(parents=True, exist_ok=True)
        os.chdir(str(build_temp))
        self.spawn([
            'cmake', '-DCMAKE_BUILD_TYPE=Release',
            f"-DBUILD_PKG_DIRECTORY={extdir}{os.sep}",
            str(ext.sourcedir)
        ])
        self.spawn([
            'cmake', '--build', '.', '--config', 'Release', '--parallel', '4'
        ])
        os.chdir(str(cwd))


setup(ext_modules=[CMakeExtension("navground_dist_py")],
      cmdclass={'build_ext': CMakeBuild})
