#pragma once

#include <glad/glad.h>

/**
 * @file shader_program.h
 * @brief Tiny RAII wrapper around an OpenGL shader program.
 */

/**
 * @brief RAII wrapper for an OpenGL shader program.
 *
 * Responsibilities:
 * - Compile vertex/fragment shaders from GLSL source strings
 * - Link them into a program
 * - Activate the program and query uniform locations
 */
class ShaderProgram
{
public:
    /**
     * @brief OpenGL program handle (0 means "not created").
     */
    GLuint programId{0};

    ShaderProgram() = default;

    /**
     * @brief Constructs and creates a program from source strings.
     * @param vsSrc Vertex shader GLSL source.
     * @param fsSrc Fragment shader GLSL source.
     */
    ShaderProgram(const char* vsSrc, const char* fsSrc);

    /**
     * @brief Deletes the program if it exists.
     */
    ~ShaderProgram();

    /**
     * @brief Creates (or recreates) the program from GLSL source strings.
     * @param vsSrc Vertex shader GLSL source.
     * @param fsSrc Fragment shader GLSL source.
     * @return True on success.
     */
    bool create(const char* vsSrc, const char* fsSrc);

    /**
     * @brief Creates (or recreates) the program by loading GLSL source from files.
     * @param vsPath Path to the vertex shader file.
     * @param fsPath Path to the fragment shader file.
     * @return True on success.
     */
    bool createFromFiles(const char* vsPath, const char* fsPath);

    /**
     * @brief Activates the program (glUseProgram).
     */
    void use() const;

    /**
     * @brief Returns the location of a uniform (or -1 if not found).
     * @param name Uniform name as declared in GLSL.
     */
    GLint loc(const char* name) const;

private:
    /**
     * @brief Compiles a single shader.
     * @param type Shader type (e.g. GL_VERTEX_SHADER).
     * @param src GLSL source.
     * @return Shader handle or 0 on failure.
     */
    static GLuint compile(GLenum type, const char* src);
};