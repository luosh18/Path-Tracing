#!/usr/bin/env bash

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    mkdir -p build
    cd build
    cmake ..
    cd ..
fi

# Build project.
cd build
make -j
cd ..

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output 
bin/PROJECT testcases/complex.txt output/complex.bmp
bin/PROJECT testcases/complex_fov.txt output/complex_fov.bmp
# bin/PROJECT testcases/scene09_norm.txt output/scene09_norm.bmp
# bin/PROJECT testcases/curve_spiral.txt output/curve_spiral.bmp
# bin/PROJECT testcases/glass_complex_dov.txt output/glass_complex_dov.bmp
# bin/PROJECT testcases/glass_complex.txt output/glass_complex.bmp
# bin/PROJECT testcases/glass_rabbit200.txt output/glass_rabbit200.bmp
# bin/PROJECT testcases/depth_of_view.txt output/depth_of_view.bmp
# bin/PROJECT testcases/diffused_spheres.txt output/diffused_spheres.bmp
# bin/PROJECT testcases/mirror_light_spheres.txt output/mirror_light_spheres.bmp


# bin/PROJECT testcases/scene00_CornellBox.txt output/scene00_CornellBox.bmp
# bin/PROJECT testcases/scene01_basic_orig.txt output/scene01_orig.bmp
# bin/PROJECT testcases/scene02_cube_orig.txt output/scene02_orig.bmp
# bin/PROJECT testcases/scene03_sphere_orig.txt output/scene03_orig.bmp
# bin/PROJECT testcases/scene04_axes_orig.txt output/scene04_orig.bmp
# bin/PROJECT testcases/scene05_bunny_200_orig.txt output/scene05_orig.bmp
# bin/PROJECT testcases/scene06_bunny_1k_orig.txt output/scene06_orig.bmp
# bin/PROJECT testcases/scene07_shine_orig.txt output/scene07_orig.bmp
