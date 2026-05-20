/**
 * @file mesh_loader.cpp
 * @brief Implementation of mesh loading via Assimp (STL & OBJ only).
 */

#include "mesh_loader.h"
#include "logger.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace graphics
{
    float MeshBounds::radius() const
    {
        const glm::vec3 kSize = max - min;
        const float kMaxExtent = std::max({std::abs(kSize.x), std::abs(kSize.y), std::abs(kSize.z)});
        return 0.5f * kMaxExtent;
    }

    namespace
    {
        void updateBounds(MeshBounds& bounds, const glm::vec3& point)
        {
            bounds.min = glm::min(bounds.min, point);
            bounds.max = glm::max(bounds.max, point);
        }

        void initBounds(MeshBounds& bounds)
        {
            bounds.min = glm::vec3(std::numeric_limits<float>::max());
            bounds.max = glm::vec3(std::numeric_limits<float>::lowest());
        }

        bool isSupportedFormat(const std::string& path)
        {
            const auto kDotPos = path.rfind('.');
            if (kDotPos == std::string::npos)
            {
                return false;
            }
            std::string ext = path.substr(kDotPos);
            std::ranges::transform(ext, ext.begin(), [](const unsigned char c) { return std::tolower(c); });
            return ext == ".stl" || ext == ".obj";
        }

        bool isValidNormal(const glm::vec3& n)
        {
            return !std::isnan(n.x) && !std::isnan(n.y) && !std::isnan(n.z) && glm::length(n) > 0.0001f;
        }

        glm::vec3 computeFaceNormal(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2)
        {
            const glm::vec3 kEdge1 = p1 - p0;
            const glm::vec3 kEdge2 = p2 - p0;
            const glm::vec3 kCross = glm::cross(kEdge1, kEdge2);
            const float kLen = glm::length(kCross);
            if (kLen < 0.0001f)
            {
                return glm::vec3(0.0f, 1.0f, 0.0f);
            }
            return kCross / kLen;
        }
    } // namespace

    MeshLoadResult MeshLoader::load(const std::string& path)
    {
        MeshLoadResult result;
        initBounds(result.bounds);

        MEDUSA_DEBUG("[MeshLoader] Starting load: '{}'", path);

        if (!isSupportedFormat(path))
        {
            MEDUSA_WARN("[MeshLoader] Unsupported format (only STL/OBJ): '{}'", path);
            return {};
        }

        Assimp::Importer importer;

        constexpr unsigned int kFlags = aiProcess_Triangulate | aiProcess_GenNormals |
            aiProcess_ValidateDataStructure | aiProcess_JoinIdenticalVertices |
            aiProcess_FixInfacingNormals;

        MEDUSA_DEBUG("[MeshLoader] Assimp flags: Triangulate|GenNormals|ValidateDataStructure|"
            "JoinIdenticalVertices|FixInfacingNormals");

        const aiScene* scene = importer.ReadFile(path, kFlags);

        if (scene == nullptr || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) != 0 || scene->mRootNode == nullptr)
        {
            MEDUSA_ERROR("[MeshLoader] Assimp failed to load '{}': {}", path, importer.GetErrorString());
            return {};
        }

        MEDUSA_DEBUG("[MeshLoader] Assimp loaded scene: {} mesh(es)", scene->mNumMeshes);

        if (scene->mNumMeshes == 0)
        {
            MEDUSA_WARN("[MeshLoader] No meshes found in '{}'", path);
            return {};
        }

        size_t totalVertices = 0;
        size_t totalTriangles = 0;
        size_t skippedFaces = 0;
        size_t degenerateTriangles = 0;

        for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
        {
            const aiMesh* mesh = scene->mMeshes[i];
            totalVertices += mesh->mNumVertices;
            totalTriangles += mesh->mNumFaces;
        }

        MEDUSA_DEBUG("[MeshLoader] Scene totals: {} vertices, {} faces", totalVertices, totalTriangles);

        result.vertices.reserve(totalTriangles * 3 * 6);

        for (unsigned int meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex)
        {
            const aiMesh* mesh = scene->mMeshes[meshIndex];
            const bool kHasNormals = mesh->HasNormals();

            MEDUSA_DEBUG("[MeshLoader] Mesh[{}]: {} vertices, {} faces, normals={}",
                         meshIndex, mesh->mNumVertices, mesh->mNumFaces, kHasNormals ? "yes" : "no");

            for (unsigned int faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex)
            {
                const aiFace& face = mesh->mFaces[faceIndex];

                if (face.mNumIndices != 3)
                {
                    ++skippedFaces;
                    continue;
                }

                const unsigned int kI0 = face.mIndices[0];
                const unsigned int kI1 = face.mIndices[1];
                const unsigned int kI2 = face.mIndices[2];

                const aiVector3D& v0 = mesh->mVertices[kI0];
                const aiVector3D& v1 = mesh->mVertices[kI1];
                const aiVector3D& v2 = mesh->mVertices[kI2];

                const glm::vec3 kP0{v0.x, v0.y, v0.z};
                const glm::vec3 kP1{v1.x, v1.y, v1.z};
                const glm::vec3 kP2{v2.x, v2.y, v2.z};

                glm::vec3 normals[3];

                if (kHasNormals)
                {
                    const aiVector3D& n0 = mesh->mNormals[kI0];
                    const aiVector3D& n1 = mesh->mNormals[kI1];
                    const aiVector3D& n2 = mesh->mNormals[kI2];
                    normals[0] = glm::vec3{n0.x, n0.y, n0.z};
                    normals[1] = glm::vec3{n1.x, n1.y, n1.z};
                    normals[2] = glm::vec3{n2.x, n2.y, n2.z};
                }
                else
                {
                    const glm::vec3 kFaceNormal = computeFaceNormal(kP0, kP1, kP2);
                    normals[0] = normals[1] = normals[2] = kFaceNormal;
                }

                for (auto& normal : normals)
                {
                    if (!isValidNormal(normal))
                    {
                        normal = computeFaceNormal(kP0, kP1, kP2);
                        ++degenerateTriangles;
                    }
                }

                const glm::vec3 kPositions[3] = {kP0, kP1, kP2};

                for (int i = 0; i < 3; ++i)
                {
                    result.vertices.insert(result.vertices.end(),
                                           {
                                               kPositions[i].x, kPositions[i].y, kPositions[i].z,
                                               normals[i].x, normals[i].y, normals[i].z
                                           });
                    updateBounds(result.bounds, kPositions[i]);
                }
            }
        }

        if (result.vertices.empty())
        {
            MEDUSA_ERROR("[MeshLoader] No valid triangles produced from '{}'", path);
            return {};
        }

        const size_t kVertexCount = result.vertexCount();
        const size_t kTriangleCount = kVertexCount / 3;

        if (kVertexCount == 0 || kTriangleCount == 0 || (kVertexCount % 3) != 0)
        {
            MEDUSA_ERROR("[MeshLoader] Validation failed: vertices={}, triangles={}", kVertexCount, kTriangleCount);
            return {};
        }

        const glm::vec3 kBoundsMin = result.bounds.min;
        const glm::vec3 kBoundsMax = result.bounds.max;
        const glm::vec3 kBoundsCenter = result.bounds.center();
        const float kBoundsRadius = result.bounds.radius();

        MEDUSA_INFO("[MeshLoader] Successfully loaded '{}'", path);
        MEDUSA_INFO("[MeshLoader]   Vertices: {}", kVertexCount);
        MEDUSA_INFO("[MeshLoader]   Triangles: {}", kTriangleCount);
        MEDUSA_INFO("[MeshLoader]   Skipped faces (non-triangle): {}", skippedFaces);
        MEDUSA_INFO("[MeshLoader]   Degenerate normals fixed: {}", degenerateTriangles);
        MEDUSA_INFO("[MeshLoader]   Bounding Box Min: ({:.3f}, {:.3f}, {:.3f})", kBoundsMin.x, kBoundsMin.y,
                    kBoundsMin.z);
        MEDUSA_INFO("[MeshLoader]   Bounding Box Max: ({:.3f}, {:.3f}, {:.3f})", kBoundsMax.x, kBoundsMax.y,
                    kBoundsMax.z);
        MEDUSA_INFO("[MeshLoader]   Bounding Box Center: ({:.3f}, {:.3f}, {:.3f})", kBoundsCenter.x, kBoundsCenter.y,
                    kBoundsCenter.z);
        MEDUSA_INFO("[MeshLoader]   Bounding Radius: {:.3f}", kBoundsRadius);

        return result;
    }

    MeshLoadResult MeshLoader::loadFromCandidates(const std::vector<std::string>& paths)
    {
        MEDUSA_DEBUG("[MeshLoader] Trying {} candidate path(s)", paths.size());

        for (const auto& path : paths)
        {
            auto result = load(path);
            if (result.isValid())
            {
                return result;
            }
        }

        MEDUSA_WARN("[MeshLoader] No valid mesh found in {} candidate(s)", paths.size());
        return {};
    }
} // namespace graphics