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
    // TO DO Implement Path Tracing Algorithm here
    float pdf = 1;
    Vector3f L_dir, L_indir;
    Intersection inter = intersect(ray);

//    if(depth){
//        float pdf_light = 1.0;
//        Intersection inter_light;
//        sampleLight(inter_light, pdf_light);
//
//        Ray r_to_lignt(ray.origin,(inter_light.coords-ray.origin).normalized());
//        Intersection inter_test = intersect(r_to_lignt);
//
//        if((inter_test.coords-inter_light.coords).norm()>0.001){ // not blocked
////            L_dir = inter_light.emit * eval
//
//        }
//    }

    // calculate shade of P surface
    if(inter.happened){
        Vector3f N = inter.normal, p = inter.coords;

        // sample on light
        Intersection inter_light;
        float pdf_light = 0.0;
        sampleLight(inter_light, pdf_light);

        Vector3f x = inter_light.coords;
        Vector3f NN = inter_light.normal;

        Ray r_to_light(p,(x-p).normalized());

        Intersection inter_test = intersect(r_to_light);    // check if light be blocked。。。也许认为灯没有碰撞体积？
        // std::cout << "x: " << x << "p: " << p << std::endl;
        // std::cout << "R: " << (x-p) << std::endl;
        // std::cout << "r_to_light: " << r_to_light.direction << std::endl;
        // std::cout << inter_test.happened << std::endl;
        Vector3f ws = r_to_light.direction * -1;    // light to p
        Vector3f wo = ray.direction;    // other to p

        // if((inter_test.coords-inter_light.coords).norm()>0.001){ // not blocked
        // std::cout << inter_light.coords << std::endl;
        // std::cout << inter_test.coords << std::endl;
        // 找到灯 && （没阻碍 || 碰撞灯） （这个假设是灯本身不触发bvh碰撞）
        if( ( pdf_light > 0 ) && (!inter_test.happened) || ((inter_test.coords - inter_light.coords).norm() < 0.001)){ // not blocked
            // std::cout << pdf_light << std::endl;
            L_dir = inter_light.emit * inter.m->eval(ws,-wo,N) * dotProduct(-ws, N) * dotProduct(ws, NN) / pow((x - p).norm(),2.0) / pdf_light;
        }

        if(get_random_float() <= RussianRoulette){ // RR_success
            Vector3f wi = inter.m->sample(wo,N);
            Ray r_to_obj(p,wi);

//            Intersection inter_obj = intersect(r_to_obj);
//            if(inter_obj.happened && !inter_obj.m->hasEmission()){
//                L_indir = castRay(r_to_obj,depth+1);
//            }
            // 这种写法 给多看到两个面但是数值爆炸
            L_indir = castRay(r_to_obj,depth+1) / inter.m->pdf(wo, wi, N) / RussianRoulette;
            // 这种写法 缺面但着色比较正确
            L_indir = castRay(r_to_obj,depth+1) * inter.m->eval(-wi,-wo,N) * dotProduct(wi, N) / inter.m->pdf(wo, wi, N) / RussianRoulette;
        }


    }

    return L_dir + L_indir;
}