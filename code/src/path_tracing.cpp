#include "path_tracing.hpp"

std::default_random_engine genarator;
std::uniform_real_distribution<float> distribution(0, 1); 

// http://www.kevinbeason.com/smallpt/
void PathTracing::Render() {
    double start = omp_get_wtime();
    int sl = int(float(samps) / (subsample*subsample) + 0.5);   // samps per AAsubsample
    float d = -float(subsample-1) / 2.0;    // for adjusting subsample position
    #pragma omp parallel for schedule(dynamic, 1) // private(specErrorCount, specCount) // OpenMP
    for (int y=0; y<height; y++) {
        fprintf(stderr, "\rRendering (%d spp) %5.2f%%", samps, 100.* y / (height - 1));
        for (int x=0; x<width; x++) {
            Vector3f pixel(0.f);
            // subsample^2 AA
            for (int sy=0; sy<subsample; sy++) {
                for (int sx=0; sx<subsample; sx++) {
                    // driffted sampling
                    Vector3f sub(0.f);
                    for (int s=0; s<sl; s++) {
                        // tent filter
                        float r1 = 2 * distribution(genarator), rx = r1<1 ? sqrt(r1)-1: 1-sqrt(2-r1);
                        float r2 = 2 * distribution(genarator), ry = r2<1 ? sqrt(r2)-1: 1-sqrt(2-r2);
                        Ray ray = camera->generateRay(Vector2f(x + (sx+d+rx), y + (sy+d+ry)));
                        sub += radiance(ray, 0) / float(samps);
                    }
                    pixel += sub / (subsample*subsample);
                }
            }
            image->SetPixel(x, y, pixel);
        }
    }
    double end = omp_get_wtime();
    int timeused = int(end - start);
    printf("\nRender Time %d min %d sec \n", timeused/60, timeused%60);
    // cout << "specCount: " << specCount << "\tspecErrorCount: " << specErrorCount <<'\n';
}

// http://www.kevinbeason.com/smallpt/
// https://www.cs.princeton.edu/courses/archive/fall16/cos526/papers/importance.pdf
Vector3f PathTracing::radiance(const Ray& ray, int depth) {
    if (depth > maxDepth) return Vector3f::ZERO;
    Hit hit;
    if (!group->intersect(ray, hit, 0)) {
        if (depth == 0) return backgroundcolor;
        if (enableGlobalLighting) {     // considering class "Light" (point light, directional light)	// ther exist bugs, do not use global illumination
            if(numlights > 0) {
				// calculate all light
                // Vector3f light(0.f);
                // for (int i=0; i<numlights; i++) {
                //     light += lights[i]->PTIllumiination(ray, group);
                // }
                // return light;
				// or ramdonly choose a light
                //return randomLight()->PTIllumiination(ray, group);
            }
        }
        return Vector3f::ZERO;
    }
    Vector3f outRay = ray.getDirection();       // the direction of the hit ray, which is the inversed direction of the return light
    Vector3f hitPoint = ray.pointAtParameter(hit.getT());
    Vector3f hitNormal = hit.getNormal();
    Material *mtl = hit.getMaterial();
    Vector3f d_color = mtl->getDiffuseColor();
    float d = mtl->getDiffuse();
    Vector3f s_color = mtl->getSpecularColor();
    float s = mtl->getSpecular();
    Vector3f r_color = mtl->getRefractColor();
    float ri = mtl->getRefractiveIndex();
    float r = mtl->getRefract();
    Vector3f e = mtl->getEmission();

    float dice = distribution(genarator); // randomly decide to go diffuse/specular/refraction	// d+s+r must <= 1 !!!
    bool into = (Vector3f::dot(outRay, hitNormal) < 0); // detected ray comes from inside or outside
    if ( (!into) || ((d+s)<=dice && dice <=(d+s+r)) ) {   // Refraction   // or when ray is from inside
        if (r > 0) {    // should calculate Refraction
            Vector3f inRayRefract = perfectRefract(outRay, hitNormal, ri);
            if (inRayRefract == Vector3f::ZERO) {   // total reflection // no r_color filter
                Vector3f inRayReflect = perfectReflect(outRay, hitNormal);
                return (into ? e : Vector3f::ZERO) + radiance(Ray(hitPoint + inRayReflect*bias, inRayReflect), depth+1); 
            }
            else { 	// refraction if into object should apply filter
                return (into ? e : Vector3f::ZERO) + (into ? r_color : 1) * radiance(Ray(hitPoint + inRayRefract*bias, inRayRefract), depth+1);
            }
        }
        return Vector3f::ZERO;
    }
    else if (into) {    // checking into is important
        if (dice < d) { // Diffuse
            Vector3f inRayDiffuse = randomDiffuse(hitNormal);
            return e + d_color * radiance(Ray(hitPoint + inRayDiffuse*bias, inRayDiffuse), depth+1);
        }
        else if(dice < (d+s)) {	// Specular
            Vector3f inRayReflect = perfectReflect(outRay, hitNormal);
            return e + s_color * radiance(Ray(hitPoint + inRayReflect*bias, inRayReflect), depth+1);
        }
    }
    return e;
}

Light* PathTracing::randomLight() {
    // randomly pickup a light
    int r1 = int(distribution(genarator) * numlights);
    r1 = r1 < numlights ? r1 : (numlights-1);
    return lights[r1];
}

// code: http://www.kevinbeason.com/smallpt/
// description: https://www.cs.princeton.edu/courses/archive/fall16/cos526/papers/importance.pdf
// demonstration http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html#sec-PointsOnSphere
Vector3f PathTracing::randomDiffuse(const Vector3f &z) {    // normal should be normalized  // cosinus distribution
    float r1 = distribution(genarator);
    float r2 = distribution(genarator); // sin^2(theta) // theta = acos(sqrt(1-r2))
    float phi = 2 * M_PI * r1;
    float r2s = sqrt(r2);       // sin(theta)
    Vector3f x = Vector3f::cross(Vector3f(r1-0.5, r2-0.5, (r1+r2)/2-0.5), z).normalized();
    Vector3f y = Vector3f::cross(z, x);
    return (x*r2s*cos(phi) + y*r2s*sin(phi) + z*sqrt(1-r2)).normalized();
}

Vector3f PathTracing::perfectReflect(const Vector3f &ray, const Vector3f &normal) {
    // return reflect ray orientaion
    return (ray - normal*2*Vector3f::dot(ray, normal)).normalized();
}

// http://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
Vector3f PathTracing::perfectRefract(const Vector3f &ray, const Vector3f &normal, float ri) {
    float cosI = Vector3f::dot(normal, ray);
    bool into = (cosI < 0);
    float etai_over_etat = into ? 1.0/ri : ri;
    cosI *= into ? (-1) : 1;
    //normal *= into ? 1 : (-1);
    float sinT2 = etai_over_etat*etai_over_etat * (1.0 - cosI*cosI);
    if (sinT2 > 1.0) return Vector3f::ZERO;
    // float cosT = sqrt(1.0 - sinT2);
    return (etai_over_etat * ray + (etai_over_etat * cosI - (sqrt(1.0 - sinT2))) * (into ? normal : -normal)).normalized();
}

void PathTracing::SaveBMP(const char *filename) {
    image->SaveBMP(filename);
}