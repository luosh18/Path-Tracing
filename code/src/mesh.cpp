#include "mesh.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>

const Vector3f AABB::AABBepsilon = Vector3f(0.01);
const float AABB::bias = 0.03;

bool KDNode::x_comp(int a, int b) {
    TriangleIndex &aIndex = this->mesh->t[a];
    TriangleIndex &bIndex = this->mesh->t[b];
    float ax = std::min(std::min(this->mesh->v[aIndex[0]][0], this->mesh->v[aIndex[1]][0]), this->mesh->v[aIndex[2]][0]);
    float bx = std::min(std::min(this->mesh->v[bIndex[0]][0], this->mesh->v[bIndex[1]][0]), this->mesh->v[bIndex[2]][0]);
    return  ax < bx;
}

bool KDNode::y_comp(int a, int b) {
    TriangleIndex &aIndex = this->mesh->t[a];
    TriangleIndex &bIndex = this->mesh->t[b];
    float ay = std::min(std::min(this->mesh->v[aIndex[0]][1], this->mesh->v[aIndex[1]][1]), this->mesh->v[aIndex[2]][1]);
    float by = std::min(std::min(this->mesh->v[bIndex[0]][1], this->mesh->v[bIndex[1]][1]), this->mesh->v[bIndex[2]][1]);
    return  ay < by;
}

bool KDNode::z_comp(int a, int b) {
    TriangleIndex &aIndex = this->mesh->t[a];
    TriangleIndex &bIndex = this->mesh->t[b];
    float az = std::min(std::min(this->mesh->v[aIndex[0]][2], this->mesh->v[aIndex[1]][2]), this->mesh->v[aIndex[2]][2]);
    float bz = std::min(std::min(this->mesh->v[bIndex[0]][2], this->mesh->v[bIndex[1]][2]), this->mesh->v[bIndex[2]][2]);
    return  az < bz;
}

void KDNode::build(int depth) {
    // builld box
    float inf = INFINITY;
    Vector3f minPoint(inf), maxPoint(-inf);
    for (int i=0; i<size; i++) {
        TriangleIndex &triIndex = mesh->t[list->at(i)];
        for (int verID=0; verID<3; verID++) {
            const Vector3f &vertex = mesh->v[triIndex[verID]];
            for (int axiID=0; axiID<3; axiID++) {
                if (vertex[axiID] < minPoint[axiID]) minPoint[axiID] = vertex[axiID];
                if (vertex[axiID] > maxPoint[axiID]) maxPoint[axiID] = vertex[axiID];
            }
        }
    }
    box = new AABB(minPoint, maxPoint);
    // printf("box built\n");
    // printf("box: "); box->min().print(); box->max().print();
    // base
    if (size == 0) {
        printf("size err\n");
        return;
    }
    // if (size <= 8) return; 
    if (size == 1) {
        TriangleIndex &triIndex = mesh->t[list->at(0)];
        Triangle *triangle = new Triangle(mesh->v[triIndex[0]],
                                          mesh->v[triIndex[1]], 
                                          mesh->v[triIndex[2]], 
                                          mesh->material);
        triangle->normal = mesh->n[list->at(0)];
        left = triangle;
        right = triangle;
        return;
    }
    if (size == 2) {
        TriangleIndex &leftIndex = mesh->t[list->at(0)];
        Triangle *leftTri = new Triangle(mesh->v[leftIndex[0]],
                                         mesh->v[leftIndex[1]], 
                                         mesh->v[leftIndex[2]], 
                                         mesh->material);
        leftTri->normal = mesh->n[list->at(0)];
        left = leftTri;

        TriangleIndex &rightIndex = mesh->t[list->at(1)];
        Triangle *rightTri = new Triangle(mesh->v[rightIndex[0]],
                                          mesh->v[rightIndex[1]], 
                                          mesh->v[rightIndex[2]], 
                                          mesh->material);
        rightTri->normal = mesh->n[list->at(1)];
        right = rightTri;
        return;
    }
    if (depth >= depth_bound) {
        triList = new std::vector<Object3D*>();
        triList->resize(size);
        for (int i=0; i<size; i++) {
            TriangleIndex &rightIndex = mesh->t[list->at(i)];
            Triangle * tri = new Triangle(mesh->v[rightIndex[0]],
                                          mesh->v[rightIndex[1]], 
                                          mesh->v[rightIndex[2]], 
                                          mesh->material);
            tri->normal = mesh->n[list->at(i)];
            triList->at(i) = tri;
        }
        return;
    }
    // not base
    // select axis to split
    int axisToSplit = box->longestAxis();
    // find midpoint of all triangles   // sort
    if (axisToSplit==0) std::sort(list->begin(), list->end(), [this](int a, int b){return x_comp(a, b);});
    else if (axisToSplit==1) std::sort(list->begin(), list->end(), [this](int a, int b){return y_comp(a, b);});
    else if (axisToSplit==2) std::sort(list->begin(), list->end(), [this](int a, int b){return z_comp(a, b);});
    else {printf("sort err\n");}
    // for each triangle, divide into left(smaller) and right(larger)
    std::vector<int>* leftSub = new std::vector<int>;
    std::vector<int>* rightSub = new std::vector<int>;
    for (int i=0; i<(size/2); i++) leftSub->push_back(list->at(i));
    for (int i=(size/2); i<size; i++) rightSub->push_back(list->at(i));

    // std::cout << "left:";
    // for (int i=0; i<leftSub->size(); i++) {
    //     std::cout << leftSub->at(i) <<",";
    // }
    // std::cout << std::endl;
    // std::cout << "right:";
    // for (int i=0; i<rightSub->size(); i++) {
    //     std::cout << rightSub->at(i) <<",";
    // }
    // std::cout << std::endl;

    left = new KDNode(mesh, leftSub, depth+1);
    right = new KDNode(mesh, rightSub, depth+1);
    // std::cout << left->size << "\t" << right->size << std::endl;
}

bool KDNode::intersect(const Ray &r, Hit &h, float tmin) {    
    Hit boxHit;
    if (box && box->intersect(r, boxHit, tmin)) {
        if (left==nullptr || right==nullptr) {
            bool result = false;
            Hit tmp;
            for (int i = 0; i < size; i++) {
                if (triList->at(i)->intersect(r, tmp, tmin)) {
                    if (tmp.getT() < h.getT()) {
                        result = true;
                        h = tmp;
                    }
                }
            }
            return result;
        }
        if (left && right) {
            if (left == right) return left->intersect(r, h, tmin);
            Hit leftHit, rightHit;
            bool isLeftHit = left->intersect(r, leftHit, tmin);
            bool isRightHit = right->intersect(r, rightHit, tmin);
            if (isLeftHit && isRightHit) {
                h = leftHit.getT() < rightHit.getT() ? leftHit : rightHit;
                return true;
            }
            else if (isLeftHit) {
                h = leftHit;
                return true;
            }
            else if (isRightHit) {
                h = rightHit;
                return true;
            }
        }
        // // if (1) {
        // else {
        //     bool result = false;
        //     Hit tmp;
        //     for (int i = 0; i < size; i++) {
        //         TriangleIndex &triIndex = mesh->t[list->at(i)];
        //         Triangle triangle(mesh->v[triIndex[0]],
        //                           mesh->v[triIndex[1]], 
        //                           mesh->v[triIndex[2]], 
        //                           mesh->material);
        //         triangle.normal = mesh->n[list->at(i)];
        //         if (triangle.intersect(r, tmp, tmin)) {
        //             if (tmp.getT() < h.getT()) {
        //                 result = true;
        //                 h = tmp;
        //             }
        //         }
        //     }
        //     return result;
        // }
    }
    return false;
}

// 原版的 Mesh 写错拉RRRRRRRRRRR S.H.I.T.
bool Mesh::intersect(const Ray &r, Hit &h, float tmin) {
    return root->intersect(r, h, tmin);

    // Optional: Change this brute force method into a faster one.
    // bool result = false;
    // Hit tmp;
    // for (int triId = 0; triId < (int) t.size(); ++triId) {
    //     TriangleIndex& triIndex = t[triId];
    //     Triangle triangle(v[triIndex[0]],
    //                       v[triIndex[1]], v[triIndex[2]], material);
    //     triangle.normal = n[triId];
    //     //result |= triangle.intersect(r, h, tmin);
    //     if (triangle.intersect(r, tmp, tmin)) {
    //         if (tmp.getT() < h.getT()) {
    //             result = true;
    //             h = tmp;
    //         }
    //     }
    // }
    // return result;
}

Mesh::Mesh(const char *filename, Material *material) : Object3D(material) {

    // Optional: Use tiny obj loader to replace this simple one.
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string texTok("vt");
    char bslash = '/', space = ' ';
    std::string tok;
    int texID;
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;
        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec);
        } else if (tok == fTok) {
            if (line.find(bslash) != std::string::npos) {
                std::replace(line.begin(), line.end(), bslash, space);
                std::stringstream facess(line);
                TriangleIndex trig;
                facess >> tok;
                for (int ii = 0; ii < 3; ii++) {
                    facess >> trig[ii] >> texID;
                    trig[ii]--;
                }
                t.push_back(trig);
            } else {
                TriangleIndex trig;
                for (int ii = 0; ii < 3; ii++) {
                    ss >> trig[ii];
                    trig[ii]--;
                }
                t.push_back(trig);
            }
        } else if (tok == texTok) {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
        }
    }
    computeNormal();

    f.close();

    int size = t.size();
    std::vector<int>* list = new std::vector<int>;
    for (int i=0; i<size; i++) list->push_back(i);
    root = new KDNode(this, list, 0);
    // if (KDNode::mesh == nullptr) printf("error!\n");
}

void Mesh::computeNormal() {
    n.resize(t.size());
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        n[triId] = b / b.length();
    }
}
