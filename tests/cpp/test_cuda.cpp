/**
 * test_cuda.cpp
 * R0.6: Verifies CUDA GPU execution on Blackwell SM 12.0.
 *
 * Tests:
 *   1. Device info (GPU found, properties accessible).
 *   2. Vector-add: GPU result matches CPU reference within 1e-6.
 *   3. Large array: 1,000,000 elements, all correct.
 */

#include "vector_add.cuh"
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

static const float TOL = 1e-6f;

bool nearly_equal_f(float a, float b, float tol = TOL) {
    return std::abs(a - b) < tol;
}

// Test 1: Device info
bool test_device_info() {
    int result = cuda_device_info();
    if (result != 0) {
        std::cerr << "FAIL: cuda_device_info returned " << result << "\n";
        return false;
    }
    std::cout << "PASS: test_device_info\n";
    return true;
}

// Test 2: Small vector-add (10 elements, known values)
bool test_vector_add_small() {
    const int n = 10;
    std::vector<float> a(n), b(n), gpu_result(n), cpu_result(n);

    for (int i = 0; i < n; ++i) {
        a[i] = static_cast<float>(i);
        b[i] = static_cast<float>(i * 2);
        cpu_result[i] = a[i] + b[i];
    }

    int ret = gpu_vector_add(a.data(), b.data(), gpu_result.data(), n);
    if (ret != 0) {
        std::cerr << "FAIL: gpu_vector_add returned " << ret << "\n";
        return false;
    }

    for (int i = 0; i < n; ++i) {
        if (!nearly_equal_f(gpu_result[i], cpu_result[i])) {
            std::cerr << "FAIL: element " << i
                      << " GPU=" << gpu_result[i]
                      << " CPU=" << cpu_result[i] << "\n";
            return false;
        }
    }

    std::cout << "PASS: test_vector_add_small\n";
    return true;
}

// Test 3: Large vector-add (1,000,000 elements)
bool test_vector_add_large() {
    const int n = 1000000;
    std::vector<float> a(n), b(n), gpu_result(n);
    float max_error = 0.0f;

    // Fill with deterministic values
    for (int i = 0; i < n; ++i) {
        a[i] = static_cast<float>(i % 1000) * 0.001f;
        b[i] = static_cast<float>((i + 500) % 1000) * 0.001f;
    }

    int ret = gpu_vector_add(a.data(), b.data(), gpu_result.data(), n);
    if (ret != 0) {
        std::cerr << "FAIL: gpu_vector_add (large) returned " << ret << "\n";
        return false;
    }

    for (int i = 0; i < n; ++i) {
        float expected = a[i] + b[i];
        float error = std::abs(gpu_result[i] - expected);
        if (error > max_error) max_error = error;
        if (error > TOL) {
            std::cerr << "FAIL: element " << i
                      << " GPU=" << gpu_result[i]
                      << " CPU=" << expected
                      << " error=" << error << "\n";
            return false;
        }
    }

    std::cout << "PASS: test_vector_add_large (n=" << n
              << ", max_error=" << max_error << ")\n";
    return true;
}

int main() {
    std::cout << "=== CUDA Tests (SM 12.0 Blackwell) ===\n\n";

    int failures = 0;

    if (!test_device_info())       { ++failures; }
    if (!test_vector_add_small())  { ++failures; }
    if (!test_vector_add_large())  { ++failures; }

    std::cout << "\n";
    if (failures == 0) {
        std::cout << "All CUDA tests passed.\n";
        return 0;
    } else {
        std::cerr << failures << " CUDA test(s) FAILED.\n";
        return 1;
    }
}
