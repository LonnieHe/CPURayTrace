//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf); // So? prefer first light mesh?

                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    const float EPS = 1e-4;
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir, L_indir;
    Intersection inter = intersect(ray);

    // calculate shade of P surface
    if(inter.happened){
        Vector3f N = inter.normal, p = inter.coords;

        Vector3f wo = ray.direction,  origin = p + N * EPS;
        // sample on light
        Intersection inter_light;
        float pdf_light = 0.0;
        if(inter.obj->hasEmit() && depth == 0){   // hit light source
            L_dir += (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f));
        }else { // sample direct light
            sampleLight(inter_light, pdf_light);

            Vector3f x = inter_light.coords;
            Vector3f NN = inter_light.normal;
            Ray r_to_light(origin,(x-origin).normalized());

            Intersection inter_test = intersect(r_to_light);    // check if light be blocked。。。也许认为灯没有碰撞体积？
            Vector3f ws = -r_to_light.direction;    // light to p

            if( ( pdf_light > 0 ) && ((!inter_test.happened) || ((inter_test.coords - inter_light.coords).norm() < 0.001))){
                float cos_phi = dotProduct(-ws, N);
                float cos_phi2 = dotProduct(ws, NN);
                L_dir = inter_light.emit * inter.m->eval(-ws,-wo,N) * cos_phi * cos_phi2 / pow((x - p).norm(),2.0) / pdf_light;
            }
        }

        if(get_random_float() <= RussianRoulette){ // RR_success
            Vector3f wi = inter.m->sample(wo,N);
            Ray r_to_obj(origin,wi);
            float cos_phi3 = dotProduct(wi, N);
            L_indir = castRay(r_to_obj,depth+1) * inter.m->eval(wi,-wo,N) * cos_phi3 / inter.m->pdf(wo, wi, N) / RussianRoulette;
        }


    }

    return L_dir + L_indir;
}