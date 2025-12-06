#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "utils.hpp"
#include "object3d.hpp"
#include <vecmath.h>
#include "triangle.hpp"
#include "curve.hpp"
#include <tuple>
#include <vector>
#include <algorithm>
#define MAX_ITERATIONS 30
//参考往届开源代码https://github.com/Guangxuan-Xiao/THU-Computer-Graphics-2020/blob/master/project/final/include/revsurface.hpp
class RevSurface : public Object3D {
    Curve *profileCurve;
    BoundingBox boundingVolume;

public:
    RevSurface(Curve *inputCurve, Material* mat) : profileCurve(inputCurve), Object3D(mat) {
        for (const auto &controlPoint : profileCurve->getControls()) {
            if (controlPoint.z() != 0.0f) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(EXIT_FAILURE);
            }
        }
        Vector3f minBound(-profileCurve->radius, profileCurve->ymin - 5, profileCurve->radius);
        Vector3f maxBound(profileCurve->radius, profileCurve->ymax + 5, profileCurve->radius);
        boundingVolume = BoundingBox(minBound, maxBound);
    }

    ~RevSurface() override {
        delete profileCurve;
    }

    void evaluateSurface(float angle, float curveParam, Vector3f &dTheta, Vector3f &dMu, Vector3f &position) {
        Quat4f rotation;
        rotation.setAxisAngle(angle, Vector3f::UP);
        Matrix3f rotationMatrix = Matrix3f::rotation(rotation);
        CurvePoint curvePt = profileCurve->getPoint(curveParam);

        dTheta = Vector3f(-curvePt.V.x() * sinf(angle), 0.0f, -curvePt.V.x() * cosf(angle));
        dMu = rotationMatrix * curvePt.T;
        position = rotationMatrix * curvePt.V;
    }

    bool performNewtonMethod(const Ray &ray, float &t, float &theta, float &mu, Vector3f &normal, Vector3f &surfacePoint) {
        Vector3f derivTheta, derivMu;
        for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
            theta = fmod(theta + 2 * M_PI, 2 * M_PI);
            mu = std::max(EPSILON, std::min(double(mu), 1.0f - EPSILON));


            evaluateSurface(theta, mu, derivTheta, derivMu, surfacePoint);
            Vector3f offset = ray.pointAtParameter(t) - surfacePoint;
            normal = Vector3f::cross(derivMu, derivTheta);

            if (offset.squaredLength() < EPSILON * EPSILON) return true;

            float denom = Vector3f::dot(ray.getDirection(), normal);
            Vector3f crossTheta = Vector3f::cross(derivTheta, offset);
            Vector3f crossMu = Vector3f::cross(derivMu, offset);

            t -= Vector3f::dot(derivMu, crossTheta) / denom;
            mu -= Vector3f::dot(ray.getDirection(), crossTheta) / denom;
            theta += Vector3f::dot(ray.getDirection(), crossMu) / denom;
        }
        return false;
    }

    bool isInvalid(float value) const {
        return std::fabs(value) < EPSILON || std::fabs(value) > 1e20f;
    }

    bool intersect(const Ray &ray, Hit &hit, float tmin, float tmax = 1e20f) override {
        float t;
        if (!boundingVolume.intersect(ray, tmin, tmax)) return false;

        float theta, mu;
        estimateInitialUV(ray, t, theta, mu);
        Vector3f normal, point;

        if (performNewtonMethod(ray, t, theta, mu, normal, point)) {
            if (isInvalid(t) || isInvalid(theta) || isInvalid(mu)) return false;
            if (t < 0 || mu < profileCurve->a || mu > profileCurve->b || t > hit.getT()) return false;

            hit.set(t, material, normal, material->getDiffuseColor());
            return true;
        }
        return false;
    }

    bool getBoundingBox(BoundingBox &outputBox) override {
        outputBox = boundingVolume;
        return true;
    }

    void estimateInitialUV(const Ray &ray, const float &t, float &theta, float &mu) {
        Vector3f projectedPoint = ray.getOrigin() + ray.getDirection() * t;
        theta = atan2(-projectedPoint.z(), projectedPoint.x()) + M_PI;
        mu = (profileCurve->ymax - projectedPoint.y()) / (profileCurve->ymax - profileCurve->ymin);
    }
};

#endif // REVSURFACE_HPP
