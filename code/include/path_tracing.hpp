#ifndef PATH_TRACING_H
#define PATH_TRACING_H

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <string>
#include <random>
#include <omp.h>
#include <exception>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"

using namespace std;

class PathTracing {
public:
    PathTracing() = delete;
    PathTracing(SceneParser* _scene): scene(_scene) {
        camera = scene->getCamera();
        width = camera->getWidth();
        height = camera->getHeight();
        backgroundcolor = scene->getBackgroundColor();
        numlights = scene->getNumLights();
        lights = new Light*[numlights];
        for (int i=0; i<numlights; i++) lights[i] = scene->getLight(i);
        nummaterials = scene->getNumMaterials();
        materials = new Material*[nummaterials];
        for (int i=0; i<nummaterials; i++) materials[i] = scene->getMaterial(i);
        group = scene->getGroup();
        image = new Image(width, height);
    }
    ~PathTracing() {
        delete image;
        delete lights;
        delete materials;
        delete scene;
    }
    void Render();
    void SaveBMP(const char *filename);

private:
    Vector3f radiance(const Ray& ray, int depth);

    Light* randomLight();
    Vector3f randomDiffuse(const Vector3f &z);
    Vector3f perfectReflect(const Vector3f &ray, const Vector3f &normal);
    Vector3f perfectRefract(const Vector3f &r, const Vector3f &n, float ri);

    const float bias = 0.001;
    const int maxDepth = 6;
    const int subsample = 2;    // 2*2 subsampling for anti-aliasing
    const int samps = 6000;       // samples for each "pixel"
    const bool enableGlobalLighting = false;    // global lighting by path tracing  // ther exist bugs, do not use global illumination
    // const bool editFileName = true;     // add render arguments to filename;

    SceneParser* scene;
    Camera* camera;
    int width;
    int height;
    Vector3f backgroundcolor;
    int numlights;
    Light** lights;
    int nummaterials;
    Material** materials;
    Group* group;
    Image* image;
};

#endif // PATH_TRACING_H
