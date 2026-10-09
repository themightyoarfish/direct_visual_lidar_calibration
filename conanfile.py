import os

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy

_EXPORT_EXCLUDES = (
    ".git",
    ".git/*",
    "build",
    "build/*",
    "build_*",
    "build_*/*",
    "build_preprocess",
    "build_preprocess/*",
    "install",
    "install/*",
    "cmake-build-*",
    ".cache",
    ".cache/*",
    ".vscode",
    ".idea",
    "__pycache__",
    "__pycache__/*",
    "*.pyc",
    ".DS_Store",
    "conanbuild.sh",
    "conanbuildenv-*",
    "conanrun.sh",
    "conanrunenv-*",
    "deactivate_conanbuild.sh",
    "deactivate_conanrun.sh",
    "compile_commands.json",
    "CMakeUserPresets.json",
)


def _set_gtsam_dir(tc, conanfile):
    try:
        gtsam = conanfile.dependencies["gtsam"]
    except (KeyError, AttributeError):
        return
    gtsam_dir = os.path.join(gtsam.package_folder, "lib", "cmake", "GTSAM")
    if os.path.isdir(gtsam_dir):
        tc.variables["GTSAM_DIR"] = gtsam_dir


class DirectVisualLidarCalibrationConan(ConanFile):
    name = "vlcal_align"
    version = "0.1.0"
    license = "MIT"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "build_vlcal_preprocess": [True, False],
        "build_with_viewer": [True, False],
        "build_with_march_native": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "build_vlcal_preprocess": False,
        "build_with_viewer": False,
        "build_with_march_native": True,
    }

    def export_sources(self):
        copy(self, "*", self.recipe_folder, self.export_sources_folder, excludes=_EXPORT_EXCLUDES)

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")
        self.options["opencv"].with_ffmpeg = False
        self.options["opencv"].with_gtk = False

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("ceres-solver/2.2.0")
        # require 4 >= opencv < 5 because opencv 5 has many API changes
        self.requires("opencv/[>=4.0.0 <5.0.0]")
        self.requires("boost/1.83.0")
        if self.options.build_vlcal_preprocess:
            self.requires("gtsam/4.3a1")
            self.requires("pcl/1.14.1")
            self.requires("fmt/10.2.1")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["BUILD_VLCAL_ALIGN"] = True
        tc.variables["BUILD_VLCAL_PREPROCESS"] = self.options.build_vlcal_preprocess
        tc.variables["BUILD_WITH_VIEWER"] = self.options.build_with_viewer
        tc.variables["BUILD_WITH_MARCH_NATIVE"] = self.options.build_with_march_native
        tc.variables["BUILD_VLCAL_TESTS"] = False
        _set_gtsam_dir(tc, self)
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_find_mode", "none")
        self.cpp_info.includedirs = ["include"]
        self.cpp_info.libdirs = ["lib"]
        self.cpp_info.libs = ["vlcal_align"]
        if self.options.build_vlcal_preprocess:
            self.cpp_info.libs.append("vlcal_preprocess")
        self.cpp_info.builddirs.append("lib/cmake/direct_visual_lidar_calibration")
