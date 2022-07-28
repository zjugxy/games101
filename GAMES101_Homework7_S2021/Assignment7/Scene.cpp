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
                objects[k]->Sample(pos, pdf);
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
    Intersection inter = intersect(ray);

    if(inter.happened==false)
        return Vector3f(0,0,0);
    if(inter.m->hasEmission())
        return inter.m->getEmission();

    Vector3f radiance_light,radiance_ref;
    Material* m = inter.m;

    Intersection lightinter;
    float lightpdf;
    sampleLight(lightinter,lightpdf);

    Vector3f N = inter.normal;
    Vector3f wo = (-1*ray.direction).normalized();
    Vector3f ws = (lightinter.coords - inter.coords).normalized();//ws 指代从接触点带选到的光源向量
    double length = (lightinter.coords - inter.coords).norm();

    Ray lightray(inter.coords,ws);
    Intersection lightray_inter = intersect(lightray);
    
    if(abs(length - lightray_inter.distance)<0.001){

        radiance_light = m->getEmission()*m->eval(wo,ws,N)*dotProduct(ws,N)
        *dotProduct(-1*ws,lightinter.normal)/lightpdf/pow(length,2);
    }

    if(get_random_float()<RussianRoulette){
        Vector3f wi = (m->sample(wo,N)).normalized();
        Intersection targetinter = intersect(Ray(inter.coords,wi));
        if(targetinter.happened==true&&!targetinter.m->hasEmission())
        {
            radiance_ref = castRay(Ray(inter.coords,wi),depth+1)*m->eval(wo,wi,N)*
            dotProduct(wi,N)/m->pdf(wo,wi,N)/RussianRoulette;
        }

    }

    return radiance_light+radiance_ref;

}