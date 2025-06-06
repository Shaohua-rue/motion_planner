#pragma once
#ifndef __UTILS_HPP
#define __UTILS_HPP

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <climits>
#include <numeric>
#include <string>
#include <vector>

namespace utils {

template <typename T>
int sign(T num) {
    if (num < 0) {
        return -1;
    }

    return 1;
}

template <typename T>
Eigen::Matrix3d transformation_matrix2d(T x, T y, T theta) {
    Eigen::Matrix3d trans;
    trans << cos(theta), -sin(theta), x, sin(theta), cos(theta), y, 0, 0, 1;

    return trans;
}

template <typename T>
Eigen::Matrix2d rotation_matrix2d(T theta) {
    Eigen::Matrix2d rotation;
    rotation << cos(theta), -sin(theta), sin(theta), cos(theta);

    return rotation;
}

template <typename T>
double pi_2_pi(T theta) {
    while (theta > M_PI) {
        theta -= 2.0 * M_PI;
    }
    while (theta < -M_PI) {
        theta += 2.0 * M_PI;
    }

    return theta;
}

template <typename T>
T max(std::vector<T> vec) {
    int size = vec.size();
    assert(size > 0);

    T ret = vec[0];
    for (int idx = 1; idx < size; ++idx) {
        if (vec[idx] > ret) {
            ret = vec[idx];
        }
    }

    return ret;
}

template <typename T>
T min(std::vector<T> vec) {
    int size = vec.size();
    assert(size > 0);

    T ret = vec[0];
    for (int idx = 1; idx < size; ++idx) {
        if (vec[idx] < ret) {
            ret = vec[idx];
        }
    }

    return ret;
}

template <typename T>
std::vector<T> diff(const std::vector<T>& vec) {
    std::vector<T> ret;
    for (size_t idx = 1; idx < vec.size(); ++idx) {
        ret.push_back(vec[idx] - vec[idx - 1]);
    }

    return ret;
}

template <typename T>
std::vector<T> cumsum(std::vector<T> vec) {
    std::vector<T> output;
    T tmp = 0;
    for (size_t idx = 0; idx < vec.size(); ++idx) {
        tmp += vec[idx];
        output.push_back(tmp);
    }

    return output;
}

template <typename T>
int search_index(std::vector<T> nums, T target) {
    int left = 0, right = nums.size() - 1;
    while (left <= right) {
        int mid = (right - left) / 2 + left;
        int num = nums[mid];
        if (num == target) {
            return mid;
        } else if (num > target) {
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }

    return -1;
}

template <typename T>
double variance(const std::vector<T>& data) {
    if (data.empty()) {
        return 0.0;
    }

    double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

    double variance = 0.0;
    for (const double& value : data) {
        variance += pow(value - mean, 2);
    }
    variance /= data.size();

    return variance;
}

class TicToc {
public:
    TicToc(void) { tic(); }

    void tic(void) { start = std::chrono::system_clock::now(); }

    double toc(void) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
}
#endif