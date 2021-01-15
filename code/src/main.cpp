#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <string>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "path_tracing.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        std::cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.

    // 1. parsing
    SceneParser* scene = new SceneParser(inputFile.c_str());
    std::cout << inputFile.c_str() << endl;
    // 2. prepare image etc.
    PathTracing* pathtracing = new PathTracing(scene);
    // 3. render image
    pathtracing->Render();
    // 4. output image
    pathtracing->SaveBMP(outputFile.c_str());
    // 5. finished
    delete pathtracing;
    std::cout << "\n----------------------------Finished!!!----------------------------\n";

    return 0;
}

