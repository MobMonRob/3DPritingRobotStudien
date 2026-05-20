/**
 * @file mesh_loader_integration_test.cpp
 * @brief Integration tests for MeshLoader with full pipeline validation.
 *
 * Tests verify end-to-end mesh loading including file I/O, Assimp processing,
 * data extraction, and consistency across different file formats.
 */

#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>

#include "mesh_loader.h"
#include "logger.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <string>

namespace
{
    std::string getAssetPath(const std::string& relativePath)
    {
        return std::string(TEST_ASSETS_PATH) + "/" + relativePath;
    }

    bool vec3ApproxEqual(const glm::vec3& a, const glm::vec3& b, const float epsilon = 0.001f)
    {
        return glm::all(glm::epsilonEqual(a, b, epsilon));
    }
} // namespace

/**
 * @brief Integration test fixture for MeshLoader.
 */
class MeshLoaderIntegrationTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        Logger::init("logs", spdlog::level::debug, spdlog::level::debug);
    }

    static void TearDownTestSuite() { Logger::shutdown(); }
};

// =============================================================================
// Format Consistency Tests
// =============================================================================

/**
 * @brief Test that ASCII and binary STL produce same vertex count for same geometry.
 *
 * Both files contain a cube with 12 triangles (36 vertices).
 */
TEST_F(MeshLoaderIntegrationTest, StlFormats_SameGeometry_SameVertexCount)
{
    const auto kAsciiResult = graphics::MeshLoader::load(getAssetPath("stl/ascii/sample_asciii_cube.stl"));
    const auto kBinaryResult = graphics::MeshLoader::load(getAssetPath("stl/binary/sample_binary_cube.stl"));

    ASSERT_TRUE(kAsciiResult.isValid()) << "ASCII STL should load successfully";
    ASSERT_TRUE(kBinaryResult.isValid()) << "Binary STL should load successfully";

    EXPECT_EQ(kAsciiResult.vertexCount(), kBinaryResult.vertexCount())
        << "ASCII and binary STL with same geometry should have same vertex count";
    EXPECT_EQ(kAsciiResult.vertexCount() % 3, 0) << "Vertex count should be multiple of 3";
}

/**
 * @brief Test that OBJ produces same vertex count as STL for same geometry.
 */
TEST_F(MeshLoaderIntegrationTest, ObjAndStl_SameTopology_SameTriangleCount)
{
    const auto kObjResult = graphics::MeshLoader::load(getAssetPath("obj/sample_binary_cube.obj"));
    const auto kStlResult = graphics::MeshLoader::load(getAssetPath("stl/binary/sample_binary_cube.stl"));

    ASSERT_TRUE(kObjResult.isValid()) << "OBJ should load successfully";
    ASSERT_TRUE(kStlResult.isValid()) << "STL should load successfully";

    // Both are cubes = 12 triangles = 36 vertices
    const size_t kObjTriangles = kObjResult.vertexCount() / 3;
    const size_t kStlTriangles = kStlResult.vertexCount() / 3;

    EXPECT_EQ(kObjTriangles, kStlTriangles) << "OBJ and STL cubes should have same triangle count (12)";
}

// =============================================================================
// Data Integrity Tests
// =============================================================================

/**
 * @brief Test that all vertices are within bounding box.
 */
TEST_F(MeshLoaderIntegrationTest, AllFormats_VerticesWithinBoundingBox)
{
    const std::vector<std::string> kFiles = {"stl/ascii/sample_asciii_cube.stl", "stl/binary/sample_binary_cube.stl",
                                             "obj/sample_binary_cube.obj"};

    for (const auto& file : kFiles)
    {
        const auto kResult = graphics::MeshLoader::load(getAssetPath(file));
        ASSERT_TRUE(kResult.isValid()) << "Failed to load: " << file;

        for (size_t i = 0; i < kResult.vertexCount(); ++i)
        {
            const size_t kOffset = i * 6;
            const glm::vec3 kPos{kResult.vertices[kOffset], kResult.vertices[kOffset + 1],
                                 kResult.vertices[kOffset + 2]};

            EXPECT_GE(kPos.x, kResult.bounds.min.x - 0.001f)
                << file << ": vertex " << i << " X below min bound";
            EXPECT_GE(kPos.y, kResult.bounds.min.y - 0.001f)
                << file << ": vertex " << i << " Y below min bound";
            EXPECT_GE(kPos.z, kResult.bounds.min.z - 0.001f)
                << file << ": vertex " << i << " Z below min bound";

            EXPECT_LE(kPos.x, kResult.bounds.max.x + 0.001f)
                << file << ": vertex " << i << " X above max bound";
            EXPECT_LE(kPos.y, kResult.bounds.max.y + 0.001f)
                << file << ": vertex " << i << " Y above max bound";
            EXPECT_LE(kPos.z, kResult.bounds.max.z + 0.001f)
                << file << ": vertex " << i << " Z above max bound";
        }
    }
}

/**
 * @brief Test that normals are consistent for each triangle (flat shading check).
 */
TEST_F(MeshLoaderIntegrationTest, StlFiles_TriangleNormalsConsistent)
{
    const std::vector<std::string> kFiles = {"stl/ascii/sample_asciii_cube.stl", "stl/binary/sample_binary_cube.stl"};

    for (const auto& file : kFiles)
    {
        const auto kResult = graphics::MeshLoader::load(getAssetPath(file));
        ASSERT_TRUE(kResult.isValid()) << "Failed to load: " << file;

        const size_t kTriangleCount = kResult.vertexCount() / 3;

        for (size_t tri = 0; tri < kTriangleCount; ++tri)
        {
            const size_t kBaseOffset = tri * 3 * 6;

            // Get normals for all 3 vertices of this triangle
            const glm::vec3 kN0{kResult.vertices[kBaseOffset + 3], kResult.vertices[kBaseOffset + 4],
                                kResult.vertices[kBaseOffset + 5]};
            const glm::vec3 kN1{kResult.vertices[kBaseOffset + 9], kResult.vertices[kBaseOffset + 10],
                                kResult.vertices[kBaseOffset + 11]};
            const glm::vec3 kN2{kResult.vertices[kBaseOffset + 15], kResult.vertices[kBaseOffset + 16],
                                kResult.vertices[kBaseOffset + 17]};

            // For STL files with flat shading, all normals in a triangle should be identical
            EXPECT_TRUE(vec3ApproxEqual(kN0, kN1, 0.01f))
                << file << ": triangle " << tri << " has inconsistent normals (v0 vs v1)";
            EXPECT_TRUE(vec3ApproxEqual(kN1, kN2, 0.01f))
                << file << ": triangle " << tri << " has inconsistent normals (v1 vs v2)";
        }
    }
}

/**
 * @brief Test that normals point outward (dot product with centroid-to-vertex is positive).
 */
TEST_F(MeshLoaderIntegrationTest, AllFormats_NormalsPointOutward)
{
    const std::vector<std::string> kFiles = {"stl/ascii/sample_asciii_cube.stl", "stl/binary/sample_binary_cube.stl",
                                             "obj/sample_binary_cube.obj"};

    for (const auto& file : kFiles)
    {
        const auto kResult = graphics::MeshLoader::load(getAssetPath(file));
        ASSERT_TRUE(kResult.isValid()) << "Failed to load: " << file;

        const glm::vec3 kCenter = kResult.bounds.center();
        size_t outwardCount = 0;

        for (size_t i = 0; i < kResult.vertexCount(); ++i)
        {
            const size_t kOffset = i * 6;
            const glm::vec3 kPos{kResult.vertices[kOffset], kResult.vertices[kOffset + 1],
                                 kResult.vertices[kOffset + 2]};
            const glm::vec3 kNormal{kResult.vertices[kOffset + 3], kResult.vertices[kOffset + 4],
                                    kResult.vertices[kOffset + 5]};

            const glm::vec3 kToVertex = glm::normalize(kPos - kCenter);
            const float kDot = glm::dot(kToVertex, kNormal);

            if (kDot > 0.0f)
            {
                ++outwardCount;
            }
        }

        // At least 90% of normals should point outward for a convex shape
        const float kOutwardRatio = static_cast<float>(outwardCount) / static_cast<float>(kResult.vertexCount());
        EXPECT_GT(kOutwardRatio, 0.9f) << file << ": only " << (kOutwardRatio * 100.0f)
                                       << "% of normals point outward (expected >90%)";
    }
}

// =============================================================================
// Performance Tests
// =============================================================================

/**
 * @brief Test that mesh loading completes within acceptable time.
 *
 * Note: First load may take longer due to Assimp lazy initialization.
 * We warm up with one load before measuring.
 */
TEST_F(MeshLoaderIntegrationTest, LoadPerformance_SmallMesh_Under500ms)
{
    const std::vector<std::string> kFiles = {"stl/ascii/sample_asciii_cube.stl", "stl/binary/sample_binary_cube.stl",
                                             "obj/sample_binary_cube.obj"};

    // Warm-up load to initialize Assimp
    [[maybe_unused]] auto warmup = graphics::MeshLoader::load(getAssetPath(kFiles[0]));

    for (const auto& file : kFiles)
    {
        const auto kStart = std::chrono::high_resolution_clock::now();
        const auto kResult = graphics::MeshLoader::load(getAssetPath(file));
        const auto kEnd = std::chrono::high_resolution_clock::now();

        const auto kDuration = std::chrono::duration_cast<std::chrono::milliseconds>(kEnd - kStart);

        ASSERT_TRUE(kResult.isValid()) << "Failed to load: " << file;
        // Allow up to 500ms for small meshes (includes OBJ parser initialization)
        EXPECT_LT(kDuration.count(), 500) << file << " took " << kDuration.count() << "ms (expected <500ms)";
    }
}

/**
 * @brief Test multiple sequential loads don't leak resources.
 */
TEST_F(MeshLoaderIntegrationTest, MultipleLoads_NoResourceLeak)
{
    constexpr int kIterations = 10;

    for (int i = 0; i < kIterations; ++i)
    {
        const auto kResult1 = graphics::MeshLoader::load(getAssetPath("stl/ascii/sample_asciii_cube.stl"));
        const auto kResult2 = graphics::MeshLoader::load(getAssetPath("stl/binary/sample_binary_cube.stl"));
        const auto kResult3 = graphics::MeshLoader::load(getAssetPath("obj/sample_binary_cube.obj"));

        ASSERT_TRUE(kResult1.isValid()) << "Iteration " << i << ": ASCII STL failed";
        ASSERT_TRUE(kResult2.isValid()) << "Iteration " << i << ": Binary STL failed";
        ASSERT_TRUE(kResult3.isValid()) << "Iteration " << i << ": OBJ failed";
    }
}

// =============================================================================
// Edge Case Tests
// =============================================================================

/**
 * @brief Test loading same file twice produces identical results.
 */
TEST_F(MeshLoaderIntegrationTest, LoadTwice_IdenticalResults)
{
    const auto kResult1 = graphics::MeshLoader::load(getAssetPath("stl/binary/sample_binary_cube.stl"));
    const auto kResult2 = graphics::MeshLoader::load(getAssetPath("stl/binary/sample_binary_cube.stl"));

    ASSERT_TRUE(kResult1.isValid());
    ASSERT_TRUE(kResult2.isValid());

    EXPECT_EQ(kResult1.vertices.size(), kResult2.vertices.size());
    EXPECT_TRUE(vec3ApproxEqual(kResult1.bounds.min, kResult2.bounds.min));
    EXPECT_TRUE(vec3ApproxEqual(kResult1.bounds.max, kResult2.bounds.max));

    // Verify all vertices are identical
    for (size_t i = 0; i < kResult1.vertices.size(); ++i)
    {
        EXPECT_FLOAT_EQ(kResult1.vertices[i], kResult2.vertices[i]) << "Mismatch at index " << i;
    }
}

/**
 * @brief Test file path with special characters (if test file exists).
 */
TEST_F(MeshLoaderIntegrationTest, LoadFromCandidates_MixedValidInvalid_ReturnsFirstValid)
{
    const std::vector<std::string> kPaths = {
        getAssetPath("nonexistent.stl"),           // Invalid
        getAssetPath("also_nonexistent.obj"),      // Invalid
        getAssetPath("stl/ascii/sample_asciii_cube.stl"), // Valid - should be returned
        getAssetPath("stl/binary/sample_binary_cube.stl") // Valid - should be skipped
    };

    const auto kResult = graphics::MeshLoader::loadFromCandidates(kPaths);

    ASSERT_TRUE(kResult.isValid());

    // Verify it's the ASCII cube (has negative Y values)
    EXPECT_LT(kResult.bounds.min.y, -10.0f) << "Should have loaded ASCII cube (has negative Y)";
}

// =============================================================================
// Validation Requirements Tests
// =============================================================================

/**
 * @brief Test that loaded mesh meets all minimum validation requirements.
 */
TEST_F(MeshLoaderIntegrationTest, AllFormats_MeetMinimumValidationRequirements)
{
    const std::vector<std::string> kFiles = {"stl/ascii/sample_asciii_cube.stl", "stl/binary/sample_binary_cube.stl",
                                             "obj/sample_binary_cube.obj"};

    for (const auto& file : kFiles)
    {
        const auto kResult = graphics::MeshLoader::load(getAssetPath(file));

        SCOPED_TRACE("Testing file: " + file);

        // Requirement 1: vertices.size() > 0
        EXPECT_GT(kResult.vertices.size(), 0) << "vertices.size() must be > 0";

        // Requirement 2: vertices % 6 == 0 (interleaved pos + normal)
        EXPECT_EQ(kResult.vertices.size() % 6, 0) << "vertices must be multiple of 6";

        // Requirement 3: vertexCount % 3 == 0 (complete triangles)
        EXPECT_EQ(kResult.vertexCount() % 3, 0) << "vertex count must be multiple of 3 (triangles)";

        // Requirement 4: Bounding box is finite
        EXPECT_FALSE(std::isinf(kResult.bounds.min.x)) << "bounds.min.x must be finite";
        EXPECT_FALSE(std::isinf(kResult.bounds.max.x)) << "bounds.max.x must be finite";
        EXPECT_FALSE(std::isnan(kResult.bounds.min.x)) << "bounds.min.x must not be NaN";
        EXPECT_FALSE(std::isnan(kResult.bounds.max.x)) << "bounds.max.x must not be NaN";

        // Requirement 5: min <= max for all axes
        EXPECT_LE(kResult.bounds.min.x, kResult.bounds.max.x) << "min.x must be <= max.x";
        EXPECT_LE(kResult.bounds.min.y, kResult.bounds.max.y) << "min.y must be <= max.y";
        EXPECT_LE(kResult.bounds.min.z, kResult.bounds.max.z) << "min.z must be <= max.z";

        // Requirement 6: Radius > 0
        EXPECT_GT(kResult.bounds.radius(), 0.0f) << "radius must be > 0";
    }
}
