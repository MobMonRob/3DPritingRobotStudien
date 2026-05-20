/**
 * @file mesh_loader_test.cpp
 * @brief Unit tests for MeshLoader (STL & OBJ import).
 *
 * Tests verify correct loading of STL (binary & ASCII) and OBJ files,
 * including vertex/normal extraction, bounding box computation, and error handling.
 */

#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>

#include "mesh_loader.h"
#include "logger.h"

#include <cmath>
#include <filesystem>
#include <string>

namespace
{
    /**
     * @brief Helper: Check if two vec3 are approximately equal.
     */
    bool vec3ApproxEqual(const glm::vec3& a, const glm::vec3& b, const float epsilon = 0.001f)
    {
        return glm::all(glm::epsilonEqual(a, b, epsilon));
    }

    /**
     * @brief Helper: Get path to a test asset.
     */
    std::string getAssetPath(const std::string& relativePath)
    {
        return std::string(TEST_ASSETS_PATH) + "/" + relativePath;
    }

    /**
     * @brief Helper: Validate mesh result has valid structure.
     */
    void validateMeshStructure(const graphics::MeshLoadResult& result)
    {
        ASSERT_TRUE(result.isValid()) << "Mesh should be valid";
        ASSERT_GT(result.vertices.size(), 0) << "Vertices should not be empty";
        EXPECT_EQ(result.vertices.size() % 6, 0) << "Vertices should be multiple of 6 (pos.xyz + normal.xyz)";
        EXPECT_EQ(result.vertexCount() * 6, result.vertices.size()) << "vertexCount() should match vertices.size()/6";
        EXPECT_EQ(result.vertexCount() % 3, 0) << "Vertex count should be multiple of 3 (triangles)";
    }

    /**
     * @brief Helper: Check all normals are unit vectors (length ~1.0).
     */
    void validateNormalsAreUnitVectors(const graphics::MeshLoadResult& result, const float epsilon = 0.01f)
    {
        for (size_t i = 0; i < result.vertexCount(); ++i)
        {
            const size_t kOffset = i * 6 + 3;
            const glm::vec3 kNormal{result.vertices[kOffset], result.vertices[kOffset + 1],
                                    result.vertices[kOffset + 2]};
            const float kLength = glm::length(kNormal);
            EXPECT_NEAR(kLength, 1.0f, epsilon) << "Normal at vertex " << i << " should be unit length, got " << kLength;
        }
    }

    /**
     * @brief Helper: Check no NaN values in vertices or normals.
     */
    void validateNoNaN(const graphics::MeshLoadResult& result)
    {
        for (size_t i = 0; i < result.vertices.size(); ++i)
        {
            EXPECT_FALSE(std::isnan(result.vertices[i])) << "NaN found at index " << i;
            EXPECT_FALSE(std::isinf(result.vertices[i])) << "Inf found at index " << i;
        }
    }
} // namespace

// =============================================================================
// Test Fixture
// =============================================================================

/**
 * @brief Test fixture for MeshLoader tests.
 *
 * Initializes logger once and provides common test asset paths.
 */
class MeshLoaderTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        Logger::init("logs", spdlog::level::debug, spdlog::level::debug);
    }

    static void TearDownTestSuite() { Logger::shutdown(); }

    // Expected values for test assets (cube 20x20x20 at origin)
    static constexpr size_t kCubeTriangleCount = 12;
    static constexpr size_t kCubeVertexCount = 36; // 12 triangles * 3 vertices

    // ASCII STL cube: 20x20x20, positioned at (0, -20, 0) to (20, 0, 20)
    static constexpr glm::vec3 kAsciiCubeBoundsMin{0.0f, -20.0f, 0.0f};
    static constexpr glm::vec3 kAsciiCubeBoundsMax{20.0f, 0.0f, 20.0f};

    // Binary STL cube: 30x30x30, positioned at (0, 0, 0) to (30, 30, 30)
    static constexpr glm::vec3 kBinaryCubeBoundsMin{0.0f, 0.0f, 0.0f};
    static constexpr glm::vec3 kBinaryCubeBoundsMax{30.0f, 30.0f, 30.0f};

    // OBJ cube: 1x1x1, positioned at (0, 0, 0) to (1, 1, 1)
    static constexpr glm::vec3 kObjCubeBoundsMin{0.0f, 0.0f, 0.0f};
    static constexpr glm::vec3 kObjCubeBoundsMax{1.0f, 1.0f, 1.0f};
};

// =============================================================================
// STL ASCII Tests
// =============================================================================

/**
 * @brief Test loading ASCII STL file successfully.
 */
TEST_F(MeshLoaderTest, LoadAsciiStl_ValidCube_ReturnsValidMesh)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("stl/ascii/sample_asciii_cube.stl"));

    validateMeshStructure(kResult);
    EXPECT_EQ(kResult.vertexCount(), kCubeVertexCount) << "Cube should have 36 vertices (12 triangles)";
}

/**
 * @brief Test ASCII STL bounding box calculation.
 */
TEST_F(MeshLoaderTest, LoadAsciiStl_ValidCube_CorrectBoundingBox)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("stl/ascii/sample_asciii_cube.stl"));

    ASSERT_TRUE(kResult.isValid());
    EXPECT_TRUE(vec3ApproxEqual(kResult.bounds.min, kAsciiCubeBoundsMin))
        << "Bounds min mismatch: got (" << kResult.bounds.min.x << ", " << kResult.bounds.min.y << ", "
        << kResult.bounds.min.z << ")";
    EXPECT_TRUE(vec3ApproxEqual(kResult.bounds.max, kAsciiCubeBoundsMax))
        << "Bounds max mismatch: got (" << kResult.bounds.max.x << ", " << kResult.bounds.max.y << ", "
        << kResult.bounds.max.z << ")";
}

/**
 * @brief Test ASCII STL normals are valid unit vectors.
 */
TEST_F(MeshLoaderTest, LoadAsciiStl_ValidCube_NormalsAreUnitVectors)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("stl/ascii/sample_asciii_cube.stl"));

    ASSERT_TRUE(kResult.isValid());
    validateNormalsAreUnitVectors(kResult);
    validateNoNaN(kResult);
}

/**
 * @brief Test ASCII STL center and radius calculation.
 */
TEST_F(MeshLoaderTest, LoadAsciiStl_ValidCube_CorrectCenterAndRadius)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("stl/ascii/sample_asciii_cube.stl"));

    ASSERT_TRUE(kResult.isValid());

    const glm::vec3 kExpectedCenter = 0.5f * (kAsciiCubeBoundsMin + kAsciiCubeBoundsMax);
    EXPECT_TRUE(vec3ApproxEqual(kResult.bounds.center(), kExpectedCenter)) << "Center should be at cube centroid";

    EXPECT_NEAR(kResult.bounds.radius(), 10.0f, 0.01f) << "Radius should be 10.0 for 20x20x20 cube";
}

// =============================================================================
// STL Binary Tests
// =============================================================================

/**
 * @brief Test loading binary STL file successfully.
 */
TEST_F(MeshLoaderTest, LoadBinaryStl_ValidCube_ReturnsValidMesh)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("stl/binary/sample_binary_cube.stl"));

    validateMeshStructure(kResult);
    EXPECT_EQ(kResult.vertexCount(), kCubeVertexCount) << "Cube should have 36 vertices (12 triangles)";
}

/**
 * @brief Test binary STL bounding box calculation.
 */
TEST_F(MeshLoaderTest, LoadBinaryStl_ValidCube_CorrectBoundingBox)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("stl/binary/sample_binary_cube.stl"));

    ASSERT_TRUE(kResult.isValid());
    EXPECT_TRUE(vec3ApproxEqual(kResult.bounds.min, kBinaryCubeBoundsMin))
        << "Bounds min mismatch: got (" << kResult.bounds.min.x << ", " << kResult.bounds.min.y << ", "
        << kResult.bounds.min.z << ")";
    EXPECT_TRUE(vec3ApproxEqual(kResult.bounds.max, kBinaryCubeBoundsMax))
        << "Bounds max mismatch: got (" << kResult.bounds.max.x << ", " << kResult.bounds.max.y << ", "
        << kResult.bounds.max.z << ")";
}

/**
 * @brief Test binary STL normals are valid unit vectors.
 */
TEST_F(MeshLoaderTest, LoadBinaryStl_ValidCube_NormalsAreUnitVectors)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("stl/binary/sample_binary_cube.stl"));

    ASSERT_TRUE(kResult.isValid());
    validateNormalsAreUnitVectors(kResult);
    validateNoNaN(kResult);
}

// =============================================================================
// OBJ Tests
// =============================================================================

/**
 * @brief Test loading OBJ file successfully.
 */
TEST_F(MeshLoaderTest, LoadObj_ValidCube_ReturnsValidMesh)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("obj/sample_binary_cube.obj"));

    validateMeshStructure(kResult);
    EXPECT_EQ(kResult.vertexCount(), kCubeVertexCount) << "Cube should have 36 vertices (12 triangles)";
}

/**
 * @brief Test OBJ bounding box calculation.
 */
TEST_F(MeshLoaderTest, LoadObj_ValidCube_CorrectBoundingBox)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("obj/sample_binary_cube.obj"));

    ASSERT_TRUE(kResult.isValid());
    EXPECT_TRUE(vec3ApproxEqual(kResult.bounds.min, kObjCubeBoundsMin))
        << "Bounds min mismatch: got (" << kResult.bounds.min.x << ", " << kResult.bounds.min.y << ", "
        << kResult.bounds.min.z << ")";
    EXPECT_TRUE(vec3ApproxEqual(kResult.bounds.max, kObjCubeBoundsMax))
        << "Bounds max mismatch: got (" << kResult.bounds.max.x << ", " << kResult.bounds.max.y << ", "
        << kResult.bounds.max.z << ")";
}

/**
 * @brief Test OBJ normals are valid unit vectors.
 */
TEST_F(MeshLoaderTest, LoadObj_ValidCube_NormalsAreUnitVectors)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("obj/sample_binary_cube.obj"));

    ASSERT_TRUE(kResult.isValid());
    validateNormalsAreUnitVectors(kResult);
    validateNoNaN(kResult);
}

/**
 * @brief Test OBJ preserves normals from file.
 */
TEST_F(MeshLoaderTest, LoadObj_ValidCube_PreservesFileNormals)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("obj/sample_binary_cube.obj"));

    ASSERT_TRUE(kResult.isValid());

    // OBJ has explicit normals; verify they are axis-aligned (cube faces)
    for (size_t i = 0; i < kResult.vertexCount(); ++i)
    {
        const size_t kOffset = i * 6 + 3;
        const glm::vec3 kNormal{kResult.vertices[kOffset], kResult.vertices[kOffset + 1],
                                kResult.vertices[kOffset + 2]};

        // Each normal should have exactly one component with abs value ~1.0, others ~0.0
        const int kNonZeroCount = (std::abs(kNormal.x) > 0.5f ? 1 : 0) + (std::abs(kNormal.y) > 0.5f ? 1 : 0) +
                                  (std::abs(kNormal.z) > 0.5f ? 1 : 0);
        EXPECT_EQ(kNonZeroCount, 1) << "Cube normals should be axis-aligned at vertex " << i;
    }
}

// =============================================================================
// Error Handling Tests
// =============================================================================

/**
 * @brief Test loading non-existent file returns invalid result.
 */
TEST_F(MeshLoaderTest, Load_NonExistentFile_ReturnsInvalid)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("nonexistent.stl"));

    EXPECT_FALSE(kResult.isValid()) << "Non-existent file should return invalid result";
    EXPECT_TRUE(kResult.vertices.empty()) << "Vertices should be empty for failed load";
}

/**
 * @brief Test loading unsupported format returns invalid result.
 */
TEST_F(MeshLoaderTest, Load_UnsupportedFormat_ReturnsInvalid)
{
    const auto kResult = graphics::MeshLoader::load(getAssetPath("stl/ascii/sample_asciii_cube.fbx"));

    EXPECT_FALSE(kResult.isValid()) << "Unsupported format should return invalid result";
}

/**
 * @brief Test loading empty path returns invalid result.
 */
TEST_F(MeshLoaderTest, Load_EmptyPath_ReturnsInvalid)
{
    const auto kResult = graphics::MeshLoader::load("");

    EXPECT_FALSE(kResult.isValid()) << "Empty path should return invalid result";
}

// =============================================================================
// loadFromCandidates Tests
// =============================================================================

/**
 * @brief Test loadFromCandidates with valid first candidate.
 */
TEST_F(MeshLoaderTest, LoadFromCandidates_FirstValid_ReturnsFirst)
{
    const std::vector<std::string> kPaths = {getAssetPath("stl/ascii/sample_asciii_cube.stl"),
                                             getAssetPath("stl/binary/sample_binary_cube.stl")};

    const auto kResult = graphics::MeshLoader::loadFromCandidates(kPaths);

    ASSERT_TRUE(kResult.isValid());
    // Should load first file (ASCII cube with min.y = -20)
    EXPECT_LT(kResult.bounds.min.y, -10.0f) << "Should have loaded ASCII cube (negative Y)";
}

/**
 * @brief Test loadFromCandidates with first invalid skips to second.
 */
TEST_F(MeshLoaderTest, LoadFromCandidates_FirstInvalid_ReturnsSecond)
{
    const std::vector<std::string> kPaths = {getAssetPath("nonexistent.stl"),
                                             getAssetPath("stl/binary/sample_binary_cube.stl")};

    const auto kResult = graphics::MeshLoader::loadFromCandidates(kPaths);

    ASSERT_TRUE(kResult.isValid());
    // Should load second file (binary cube with min = 0,0,0)
    EXPECT_TRUE(vec3ApproxEqual(kResult.bounds.min, kBinaryCubeBoundsMin));
}

/**
 * @brief Test loadFromCandidates with all invalid returns invalid.
 */
TEST_F(MeshLoaderTest, LoadFromCandidates_AllInvalid_ReturnsInvalid)
{
    const std::vector<std::string> kPaths = {getAssetPath("nonexistent1.stl"), getAssetPath("nonexistent2.stl")};

    const auto kResult = graphics::MeshLoader::loadFromCandidates(kPaths);

    EXPECT_FALSE(kResult.isValid());
}

/**
 * @brief Test loadFromCandidates with empty list returns invalid.
 */
TEST_F(MeshLoaderTest, LoadFromCandidates_EmptyList_ReturnsInvalid)
{
    const auto kResult = graphics::MeshLoader::loadFromCandidates({});

    EXPECT_FALSE(kResult.isValid());
}

// =============================================================================
// MeshBounds Tests
// =============================================================================

/**
 * @brief Test MeshBounds center calculation.
 */
TEST(MeshBoundsTest, Center_SymmetricBounds_ReturnsOrigin)
{
    graphics::MeshBounds bounds;
    bounds.min = glm::vec3(-10.0f, -10.0f, -10.0f);
    bounds.max = glm::vec3(10.0f, 10.0f, 10.0f);

    EXPECT_TRUE(vec3ApproxEqual(bounds.center(), glm::vec3(0.0f)));
}

/**
 * @brief Test MeshBounds center with asymmetric bounds.
 */
TEST(MeshBoundsTest, Center_AsymmetricBounds_ReturnsCorrectCenter)
{
    graphics::MeshBounds bounds;
    bounds.min = glm::vec3(0.0f, 0.0f, 0.0f);
    bounds.max = glm::vec3(20.0f, 40.0f, 60.0f);

    EXPECT_TRUE(vec3ApproxEqual(bounds.center(), glm::vec3(10.0f, 20.0f, 30.0f)));
}

/**
 * @brief Test MeshBounds radius calculation.
 */
TEST(MeshBoundsTest, Radius_CubeBounds_ReturnsHalfMaxExtent)
{
    graphics::MeshBounds bounds;
    bounds.min = glm::vec3(0.0f, 0.0f, 0.0f);
    bounds.max = glm::vec3(20.0f, 20.0f, 20.0f);

    EXPECT_NEAR(bounds.radius(), 10.0f, 0.001f);
}

/**
 * @brief Test MeshBounds radius with non-uniform bounds.
 */
TEST(MeshBoundsTest, Radius_RectangularBounds_ReturnsHalfLargestExtent)
{
    graphics::MeshBounds bounds;
    bounds.min = glm::vec3(0.0f, 0.0f, 0.0f);
    bounds.max = glm::vec3(10.0f, 20.0f, 30.0f);

    // Largest extent is Z (30), so radius = 15
    EXPECT_NEAR(bounds.radius(), 15.0f, 0.001f);
}

// =============================================================================
// MeshLoadResult Tests
// =============================================================================

/**
 * @brief Test MeshLoadResult isValid with empty vertices.
 */
TEST(MeshLoadResultTest, IsValid_EmptyVertices_ReturnsFalse)
{
    graphics::MeshLoadResult result;
    EXPECT_FALSE(result.isValid());
}

/**
 * @brief Test MeshLoadResult isValid with vertices.
 */
TEST(MeshLoadResultTest, IsValid_WithVertices_ReturnsTrue)
{
    graphics::MeshLoadResult result;
    result.vertices = {1.0f, 2.0f, 3.0f, 0.0f, 1.0f, 0.0f};
    EXPECT_TRUE(result.isValid());
}

/**
 * @brief Test MeshLoadResult vertexCount calculation.
 */
TEST(MeshLoadResultTest, VertexCount_MultipleVertices_ReturnsCorrectCount)
{
    graphics::MeshLoadResult result;
    // 3 vertices * 6 floats each
    result.vertices.resize(18);
    EXPECT_EQ(result.vertexCount(), 3);
}
