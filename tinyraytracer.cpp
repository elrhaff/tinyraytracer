#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <algorithm>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "model.h"

int envmap_width, envmap_height;
std::vector<Vec3f> envmap;
//Model duck("../duck.obj");
constexpr int nImages = 200;


struct Material {
    Material(const float r, const Vec4f &a, const Vec3f &color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

Material      ivory(1.0, Vec4f(0.6,  0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3),   50.);
Material      glass(1.5, Vec4f(0.0,  0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8),  125.);
Material      red_rubber(1.0, Vec4f(0.9,  0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1),   10.);
Material      mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);

struct Light {
    Light(const Vec3f &p, const float i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};


struct Sphere {
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L*dir;
        float d2 = L*L - tca*tca;
        if (d2 > radius*radius) return false;
        float thc = sqrtf(radius*radius - d2);
        t0       = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        return t0 >= 0;
    }
};

Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}

Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Snell's law
    float cosi = - std::max(-1.f, std::min(1.f, I*N));
    if (cosi<0) return refract(I, -N, eta_i, eta_t); // if the ray comes from the inside the object, swap the air and the media
    float eta = eta_i / eta_t;
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return k<0 ? Vec3f(1,0,0) : I * eta + N * (eta*cosi - sqrtf(k)); // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
}

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (const auto & sphere : spheres) {
        float dist_i;
        if (sphere.ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - sphere.center).normalize();
            material = sphere.material;
        }
    }

    float checkerboard_dist = std::numeric_limits<float>::max();
//    if (fabs(dir.y)>1e-3)  {
//        float d = -(orig.y+4)/dir.y; // the checkerboard plane has equation y = -4
//        Vec3f pt = orig + dir * d;
//        if (d>0 && fabs(pt.x)<10 && pt.z<-10 && pt.z>-30 && d<spheres_dist) {
//            checkerboard_dist = d;
//            hit = pt;
//            N = Vec3f(0,1,0);
//            material.diffuse_color = (int(0.5 * hit.x + 1000) + int(0.5 * hit.z)) & 1u ? Vec3f(.3, .3, .3) : Vec3f(.3, .2, .1);
//        }
//    }
//
    float duck_dist = std::numeric_limits<float>::max();
//    if(duck.ray_box_intersect(orig, dir)){
//        float dist_duck_i;
//        for(int i = 0; i < duck.nfaces(); i++){
//            if(duck.ray_box_intersect(orig, dir)){
//                if(duck.ray_triangle_intersect(i, orig, dir, dist_duck_i, hit, N) && dist_duck_i < duck_dist && dist_duck_i < spheres_dist && dist_duck_i < checkerboard_dist){
//                    duck_dist = dist_duck_i;
//                    material = glass;
//                }
//            }
//
//
//        }
//    }

//    return std::min(duck_dist, std::min(spheres_dist, checkerboard_dist))<1000;
    return std::min(duck_dist, std::min(spheres_dist, checkerboard_dist))<1000;

}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, const std::vector<Vec3f> &background, const double &theta, size_t depth=0) {
    Vec3f point, N;
    Material material;

    if (depth> 10 || !scene_intersect(orig, dir, spheres, point, N, material)) {
        // background
        // int x_raw = (dir.x/2 + 0.5) * envmap_width;
        // int y_raw = -(dir.y/2 + 0.5) * envmap_height;
        int x_raw = ((int) ((atan2(dir.z, dir.x) / (2 * M_PI) + 0.5) * envmap_width) + (int) (theta * 1.f * envmap_width / (2 * M_PI))) % envmap_width;
        int y_raw = (int) (acos(dir.y) / M_PI * envmap_height);
        int x = std::max(0, std::min(x_raw, envmap_width -1));
        int y = std::max(0, std::min(y_raw, envmap_height -1));
        return background[x + y * envmap_width];
}


    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // offset the original point to avoid occlusion by the object itself
    Vec3f refract_orig = refract_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, background, theta, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, background, theta, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (auto light : lights) {
        Vec3f light_dir      = (light.position - point).normalize();
        float light_distance = (light.position - point).norm();

        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity  += light.intensity * std::max(0.f, light_dir*N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*light.intensity;
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];
}

void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights, const std::vector<Vec3f> &background, const int &step) {
    const int   width    = 720;
    const int   height   = 480;
//    const int   width    = 1024;
//    const int   height   = 768;
    const float fov      = M_PI/3.;
    std::vector<Vec3f> framebuffer(width * height);

    double theta = 2.f * M_PI * step / nImages;


#pragma omp parallel for
    for (size_t j = 0; j<height; j++) { // actual rendering loop
        for (size_t i = 0; i<width; i++) {
            float dir_x =  (i + 0.5f) -  width/2.f;
            float dir_y = -(j + 0.5f) + height/2.f;    // this flips the image at the same time
            float dir_z = -height/(2.0f * tan(fov/2.f));
            framebuffer[i+j*width] = cast_ray(Vec3f(0,0,0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights, background, theta);
        }
    }

    std::vector<unsigned char> pixmap(width*height*3);
    for (size_t i = 0; i < height*width; ++i) {
        Vec3f &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max>1) c = c*(1./max);
        for (size_t j = 0; j<3; j++) {
            pixmap[i*3+j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    std::string fileName = "../output/out_" + std::to_string(step) + ".jpg";
    stbi_write_jpg(fileName.c_str(), width, height, 3, pixmap.data(), 100);
}

int main() {
    int n = -1;
    unsigned char *pixmap = stbi_load("../360.jpg", &envmap_width, &envmap_height, &n, 0);
    if (!pixmap || 3!=n) {
        std::cerr << "Error: can not load the environment map" << std::endl;
        return -1;
    }
    envmap = std::vector<Vec3f>(envmap_width * envmap_height);
    for (int j = envmap_height-1; j>=0 ; j--) {
        for (int i = 0; i<envmap_width; i++) {
            envmap[i+j*envmap_width] = Vec3f(pixmap[(i+j*envmap_width)*3+0], pixmap[(i+j*envmap_width)*3+1], pixmap[(i+j*envmap_width)*3+2])*(1/255.);
        }
    }
    stbi_image_free(pixmap);

    std::vector<Sphere> spheres;
    spheres.emplace_back(Sphere(Vec3f(-3,    0,   -16), 2,      mirror));
    spheres.emplace_back(Sphere(Vec3f(-1.0, -1.5, -12), 2,      mirror));
    spheres.emplace_back(Sphere(Vec3f( 1.5, -0.5, -18), 3, mirror));
    spheres.emplace_back(Sphere(Vec3f( 7,    5,   -18), 4,     mirror));

    std::vector<Light>  lights;
    lights.emplace_back(Light(Vec3f(-20, 20,  20), 1.5));
    lights.emplace_back(Light(Vec3f( 30, 50, -25), 1.8));
    lights.emplace_back(Light(Vec3f( 30, 20,  30), 1.7));
    for(size_t i = 0; i < nImages; i++){
        render(spheres, lights, envmap, i);
    }

    return 0;
}

