#include <iostream>
#include <math.h>
#include "Vector3f.h"
#include "utils.hpp"

class PhaseFunction {
private:
    double g; // 各向异性参数 [-1, 1]
    
public:
    PhaseFunction(double anisotropy = 0.0) : g(anisotropy) {}
    
    // 计算相函数值
    double evaluate(const Vector3f& wi, const Vector3f& wo) const {
        double cos_theta = wi.dot(wo);
        double denom = 1.0 + g * g - 2.0 * g * cos_theta;
        return (1.0 - g * g) / (4.0 * M_PI * pow(denom, 1.5));
    }
    
    // 采样新的散射方向
    Vector3f sample(const Vector3f& wi) const {
        double cos_theta;
        if (abs(g) < 1e-3) {
            // 各向同性散射
            cos_theta = 1.0 - 2.0 * rng.uniform();
        } else {
            // Henyey-Greenstein采样
            double sqr_term = (1.0 - g * g) / (1.0 - g + 2.0 * g * rng.uniform());
            cos_theta = (1.0 + g * g - sqr_term * sqr_term) / (2.0 * g);
        }
        
        double sin_theta = sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));
        double phi = 2.0 * M_PI * u(mt);
        
        // 构建局部坐标系
        Vector3f w = wi * -1.0; // 入射方向的反向
        Vector3f u = (abs(w.x) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0));
        u = Vector3f(u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x).normalize();
        Vector3f v = Vector3f(w.y * u.z - w.z * u.y, w.z * u.x - w.x * u.z, w.x * u.y - w.y * u.x);
        
        return (u * (sin_theta * cos(phi)) + v * (sin_theta * sin(phi)) + w * cos_theta).normalize();
    }
};

// 体积传输模拟器
class VolumeRenderer {
private:
    MediumProperties medium;
    PhaseFunction phase_func;
    Random rng;
    
public:
    VolumeRenderer(const MediumProperties& med, const PhaseFunction& phase) 
        : medium(med), phase_func(phase) {}
    
    // 计算透射率 T(x,y) = exp(-∫μt ds)
    double transmittance(double distance) const {
        return exp(-medium.extinction_coeff * distance);
    }
    
    // 计算光学厚度 τ(x,y) = ∫μt ds
    double optical_thickness(double distance) const {
        return medium.extinction_coeff * distance;
    }
    
    // 采样距离 - 指数分布采样
    double sample_distance() {
        if (medium.extinction_coeff <= 0) return INFINITY;
        return -log(1.0 - rng.uniform()) / medium.extinction_coeff;
    }
    
    // 采样距离的概率密度函数
    double distance_pdf(double distance) const {
        return medium.extinction_coeff * transmittance(distance);
    }
    
    // Monte Carlo体积渲染 - 路径追踪实现
    Vector3f volume_render(const Ray& ray, double max_distance, 
                      std::function<Vector3f(const Vector3f&)> emission_func,
                      std::function<Vector3f(const Vector3f&, const Vector3f&)> background_func,
                      int max_bounces = 10) {
        
        Vector3f radiance(0, 0, 0);
        Vector3f throughput(1, 1, 1); // 路径权重
        
        Vector3f current_pos = ray.origin;
        Vector3f current_dir = ray.direction;
        double traveled_distance = 0;
        
        for (int bounce = 0; bounce < max_bounces; ++bounce) {
            // 采样到下一个散射事件的距离
            double sampled_distance = sample_distance();
            double total_distance = traveled_distance + sampled_distance;
            
            // 检查是否超出体积边界
            if (total_distance >= max_distance) {
                // 光线离开体积，计算背景贡献
                double remaining_distance = max_distance - traveled_distance;
                Vector3f exit_pos = current_pos + current_dir * remaining_distance;
                
                radiance = radiance + throughput * background_func(exit_pos, current_dir) * 
                          transmittance(remaining_distance);
                break;
            }
            
            // 更新位置
            current_pos = current_pos + current_dir * sampled_distance;
            traveled_distance = total_distance;
            
            // 添加发射贡献
            radiance = radiance + throughput * emission_func(current_pos) * 
                      medium.absorption_coeff / medium.extinction_coeff;
            
            // 检查是否发生散射
            if (rng.uniform() < medium.albedo()) {
                // 发生散射 - 采样新方向
                current_dir = phase_func.sample(current_dir, rng);
                
                // 更新路径权重
                throughput = throughput * medium.albedo();
                
                // 俄罗斯轮盘赌终止
                if (bounce > 3) {
                    double survival_prob = std::min(0.95, 
                        std::max({throughput.x, throughput.y, throughput.z}));
                    if (rng.uniform() > survival_prob) break;
                    throughput = throughput / survival_prob;
                }
            } else {
                // 光子被吸收
                break;
            }
        }
        
        return radiance;
    }
    
    // Delta tracking (Woodcock tracking) 实现
    Vector3f delta_tracking_render(const Ray& ray, double max_distance,
                              std::function<double(const Vector3f&)> density_func,
                              std::function<Vector3f(const Vector3f&)> emission_func,
                              std::function<Vector3f(const Vector3f&, const Vector3f&)> background_func,
                              double max_density, int max_bounces = 10) {
        
        Vector3f radiance(0, 0, 0);
        Vector3f throughput(1, 1, 1);
        
        Vector3f current_pos = ray.origin;
        Vector3f current_dir = ray.direction;
        double traveled_distance = 0;
        
        for (int bounce = 0; bounce < max_bounces; ++bounce) {
            while (traveled_distance < max_distance) {
                // 采样到下一个事件的距离
                double dt = -log(1.0 - rng.uniform()) / max_density;
                traveled_distance += dt;
                
                if (traveled_distance >= max_distance) {
                    // 到达边界
                    Vector3f exit_pos = ray.at(max_distance);
                    radiance = radiance + throughput * background_func(exit_pos, current_dir);
                    return radiance;
                }
                
                current_pos = ray.at(traveled_distance);
                double local_density = density_func(current_pos);
                
                // 接受/拒绝采样
                if (rng.uniform() < local_density / max_density) {
                    // 真实交互事件
                    double local_albedo = medium.scattering_coeff / 
                                         (medium.absorption_coeff + medium.scattering_coeff);
                    
                    // 添加发射
                    radiance = radiance + throughput * emission_func(current_pos) * 
                              medium.absorption_coeff;
                    
                    if (rng.uniform() < local_albedo) {
                        // 散射
                        current_dir = phase_func.sample(current_dir, rng);
                        throughput = throughput * local_albedo;
                    } else {
                        // 吸收
                        return radiance;
                    }
                    break;
                }
                // 否则是虚拟事件，继续采样
            }
        }
        
        return radiance;
    }
};
