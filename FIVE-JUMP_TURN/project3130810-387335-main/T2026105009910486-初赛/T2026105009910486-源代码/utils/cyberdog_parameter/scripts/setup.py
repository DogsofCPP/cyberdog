import os
import parser
import sysconfig

from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext

def get_ext_filename_without_platform_suffix(filename):
    name, ext = os.path.splitext(filename)
    ext_suffix = sysconfig.get_config_var('EXT_SUFFIX')

    if ext_suffix == ext:
        return filename

    ext_suffix = ext_suffix.replace(ext, '')
    idx = name.find(ext_suffix)

    if idx == -1:
        return filename
    else:
        return name[:idx] + ext

class BuildExtWithoutPlatformSuffix(build_ext):
    def get_ext_filename(self, ext_name):
        filename = super().get_ext_filename(ext_name)
        return get_ext_filename_without_platform_suffix(filename)

def GenertateTmpFile():
    files_list = os.listdir(os.getcwd())
    for filename in files_list:
        if os.path.splitext(filename)[1] == '.toml':
            print(filename)
            f = parser.FileOperation()
            tdata = f.read(filename)
            f.generate_hpp(tdata)


def GenerateSharedLibrary():
    files_list = os.listdir(os.getcwd())
    for filename in files_list:
        if os.path.splitext(filename)[1] == '.pyx':
            module_name = str(filename).split(".", 1)[0]
            ext = Extension(str(module_name), sources=[filename])
            setup(name=str(module_name), cmdclass={'build_ext': BuildExtWithoutPlatformSuffix}, ext_modules = cythonize([ext]))



if __name__ == '__main__':
    GenertateTmpFile()
    GenerateSharedLibrary()