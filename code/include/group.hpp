#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>


// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group() {
        size = 0;
    }

    explicit Group (int num_objects) {
        size = 0;
        elements.resize(num_objects);
    }

    ~Group() override {
        for (int i=0; i<size; i++) {
            delete elements[i];
        }
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        Hit tmp_hit;    // 用以存放正在检查的物件
        h = tmp_hit;
        bool flag = false;
        for (int i=0; i<size; i++) {
            if (elements[i]->intersect(r, tmp_hit, tmin)) {
                flag = true;
                if(tmp_hit.getT() < h.getT()){
                    h = tmp_hit;
                }
            }
        }
        return flag;
    }

    void addObject(int index, Object3D *obj) {
        elements[index] = obj;
        size++;
    }

    int getGroupSize() {
        return size;
    }

private:
    std::vector<Object3D*> elements;
    int size;
};

#endif
	
