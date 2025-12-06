#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <cmath>
#include <cassert>
#include <algorithm>

#define MAX_EPS 1e20f
//参考往届开源代码https://github.com/Guangxuan-Xiao/THU-Computer-Graphics-2020/blob/master/project/final/include/revsurface.hpp

struct CurvePoint {
    Vector3f V; // Vertex
    Vector3f T; // Tangent (unit vector)
};

class Curve : public Object3D {
public:
    float ystart, yend, radius;
    float a, b;

    explicit Curve(std::vector<Vector3f> pts)
        : controls(std::move(pts)), ystart(MAX_EPS), yend(-MAX_EPS), radius(0.0f) {
        for (const auto& p : controls) {
            ystart = std::min(ystart, p.y());
            yend = std::max(yend, p.y());
            radius = std::max({ radius, std::abs(p.x()), std::abs(p.z()) });
        }
    }

    // Main interface to evaluate a point and tangent on the curve
    virtual CurvePoint getPoint(float t) {
        return evaluate(t);
    }

    // Accessor for control points
    std::vector<Vector3f> getControls() const {
        return controls;
    }

    bool getBoundingBox(BoundingBox&) override { return false; }
    bool intersect(const Ray&, Hit&, float, float = MAX_EPS) override { return false; }

protected:
    std::vector<Vector3f> controls;

    // Unified internal evaluator
    virtual CurvePoint evaluate(float t) {
        return { Vector3f(0), Vector3f(0) };
    }
};

// ============ Bezier Curve ============
class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vector3f>& pts)
        : Curve(pts) {
        assert(pts.size() >= 2);
        a = 0.0f;
        b = 1.0f;
    }

    CurvePoint getPoint(float t) override {
        return evaluate(t);
    }

protected:
    CurvePoint evaluate(float t) override {
        int n = controls.size() - 1;
        Vector3f V(0), T(0);

        auto binomial = [](int k, int n) {
            float res = 1.0f;
            for (int i = 1; i <= k; ++i)
                res *= float(n - (k - i)) / i;
            return res;
        };

        for (int i = 0; i <= n; ++i) {
            float coeff = binomial(i, n) * std::pow(t, i) * std::pow(1 - t, n - i);
            V += coeff * controls[i];

            if (i < n) {
                float dcoeff = binomial(i, n - 1) * std::pow(t, i) * std::pow(1 - t, n - 1 - i);
                T += dcoeff * (controls[i + 1] - controls[i]) * n;
            }
        }

        return { V, T.normalized() };
    }
};

// ============ B-Spline Curve ============
class BsplineCurve : public Curve {
public:
    BsplineCurve(const std::vector<Vector3f>& pts, int deg = 3)
        : Curve(pts), degree(deg) {
        assert(pts.size() > degree);
        initKnots();
        a = knots[degree];
        b = knots.back();
    }

    CurvePoint getPoint(float t) override {
        return evaluate(t);
    }

protected:
    CurvePoint evaluate(float t) override {
        Vector3f V(0), T(0);
        int n = controls.size() - 1;

        for (int i = 0; i <= n; ++i) {
            auto [bval, dval] = basisAndDerivative(i, degree, t);
            V += bval * controls[i];
            if (i < n)
                T += dval * (controls[i + 1] - controls[i]);
        }

        return { V, T.normalized() };
    }

private:
    int degree;
    std::vector<float> knots;

    void initKnots() {
        int m = controls.size() - 1 + degree + 1;
        knots.resize(m + 1);
        for (int i = 0; i <= m; ++i)
            knots[i] = float(i) / m;
    }

    std::pair<float, float> basisAndDerivative(int i, int p, float t) {
        if (p == 0) {
            return { (t >= knots[i] && t < knots[i + 1]) ? 1.0f : 0.0f, 0.0f };
        }

        float denom1 = knots[i + p] - knots[i];
        float denom2 = knots[i + p + 1] - knots[i + 1];

        float b1 = 0, d1 = 0, b2 = 0, d2 = 0;

        if (denom1 > 1e-6f) {
            auto [b, d] = basisAndDerivative(i, p - 1, t);
            b1 = (t - knots[i]) / denom1 * b;
            d1 = (b + (t - knots[i]) * d) / denom1;
        }

        if (denom2 > 1e-6f) {
            auto [b, d] = basisAndDerivative(i + 1, p - 1, t);
            b2 = (knots[i + p + 1] - t) / denom2 * b;
            d2 = (-b + (knots[i + p + 1] - t) * d) / denom2;
        }

        return { b1 + b2, d1 + d2 };
    }
};

#endif // CURVE_HPP
