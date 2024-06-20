## CSE_167 Raytracer

Raytracer implementation from scratch. (CSE167 WINTER 2024)

# Features

    - Scene file Parser
    - Customized math library for vector and matrix calculation
    - Ray-triangle intersection calculation
    - Ray-sphere intersection calculation
    - Color calculation
    - Applies Bounding Volume Hierarchy(BVH) acceleration structure

# Flow

    1. 'readfile' parses scene files
    2. 'Raytracer' instantiates a 'BVH'
    3. 'Raytracer' calculate intersections
    4. 'Raytracer' calculate lightColors and stores them in a global 2d pixel array
    5. Generate output image using FreeIamge.dll

# Build
    - Built with VS2022
    - for testscenes 6 and 7, run with 'release' option will significantly reduce the execution time

# Contributors
    [Bran Zhang](https://github.com/kaijia2022)
    [Henry Feng](https://github.com/Henryfzh)

