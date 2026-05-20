#include <string>

#include "logger.h"
#include "shader_program.h"

/**
 * @file shader_program.cpp
 * @brief Implementation of @ref ShaderProgram.
 */

namespace
{
    /**
     * @brief Returns a readable shader type name for logging.
     */
    const char* shaderTypeToString(const GLenum shaderType)
    {
        switch (shaderType)
        {
        case GL_VERTEX_SHADER:
            return "vertex";
        case GL_FRAGMENT_SHADER:
            return "fragment";
        default:
            return "unknown";
        }
    }

    /**
     * @brief Retrieves and returns a shader's info log.
     */
    std::string getShaderInfoLog(const GLuint shaderHandle)
    {
        GLint infoLogLength = 0;
        glGetShaderiv(shaderHandle, GL_INFO_LOG_LENGTH, &infoLogLength);
        if (infoLogLength <= 1)
            return {};

        std::string infoLog(static_cast<size_t>(infoLogLength), '\0');
        GLsizei written = 0;
        glGetShaderInfoLog(shaderHandle, infoLogLength, &written, infoLog.data());
        if (written > 0 && static_cast<size_t>(written) < infoLog.size())
            infoLog.resize(static_cast<size_t>(written));

        return infoLog;
    }

    /**
     * @brief Retrieves and returns a program's info log.
     */
    std::string getProgramInfoLog(const GLuint programHandle)
    {
        GLint infoLogLength = 0;
        glGetProgramiv(programHandle, GL_INFO_LOG_LENGTH, &infoLogLength);
        if (infoLogLength <= 1)
            return {};

        std::string infoLog(static_cast<size_t>(infoLogLength), '\0');
        GLsizei written = 0;
        glGetProgramInfoLog(programHandle, infoLogLength, &written, infoLog.data());
        if (written > 0 && static_cast<size_t>(written) < infoLog.size())
            infoLog.resize(static_cast<size_t>(written));

        return infoLog;
    }
} // namespace

ShaderProgram::ShaderProgram(const char* vsSrc, const char* fsSrc)
{
    create(vsSrc, fsSrc);
}

ShaderProgram::~ShaderProgram()
{
    if (programId)
    {
        MEDUSA_DEBUG("Deleting shader program id={}", programId);
        glDeleteProgram(programId);
    }
}

GLuint ShaderProgram::compile(const GLenum shaderType, const char* shaderSource)
{
    if (!shaderSource)
    {
        MEDUSA_ERROR("Shader source is null (type={})", shaderTypeToString(shaderType));
        return 0;
    }

    const GLuint kShaderHandle = glCreateShader(shaderType);
    glShaderSource(kShaderHandle, 1, &shaderSource, nullptr);
    glCompileShader(kShaderHandle);

    GLint compileSucceeded = 0;
    glGetShaderiv(kShaderHandle, GL_COMPILE_STATUS, &compileSucceeded);
    if (!compileSucceeded)
    {
        const std::string kInfoLog = getShaderInfoLog(kShaderHandle);
        MEDUSA_ERROR("Shader compile failed (type={}): {}", shaderTypeToString(shaderType), kInfoLog);

        glDeleteShader(kShaderHandle);
        return 0;
    }

    // Only log at DEBUG level to keep startup clean.
    const std::string kInfoLog = getShaderInfoLog(kShaderHandle);
    if (!kInfoLog.empty())
        MEDUSA_DEBUG("Shader compile succeeded with warnings (type={}): {}", shaderTypeToString(shaderType), kInfoLog);

    return kShaderHandle;
}

bool ShaderProgram::create(const char* vsSrc, const char* fsSrc)
{
    if (programId)
    {
        MEDUSA_DEBUG("Recreating shader program: deleting previous id={}", programId);
        glDeleteProgram(programId);
        programId = 0;
    }

    MEDUSA_INFO("Creating shader program");

    const GLuint kVertexShaderHandle = compile(GL_VERTEX_SHADER, vsSrc);
    if (!kVertexShaderHandle)
    {
        MEDUSA_ERROR("Failed to compile vertex shader");
        return false;
    }

    const GLuint kFragmentShaderHandle = compile(GL_FRAGMENT_SHADER, fsSrc);
    if (!kFragmentShaderHandle)
    {
        MEDUSA_ERROR("Failed to compile fragment shader");
        glDeleteShader(kVertexShaderHandle);
        return false;
    }

    programId = glCreateProgram();
    glAttachShader(programId, kVertexShaderHandle);
    glAttachShader(programId, kFragmentShaderHandle);

    // The renderer expects fixed attribute locations.
    glBindAttribLocation(programId, 0, "aPos");
    glBindAttribLocation(programId, 1, "aNormal");

    glLinkProgram(programId);

    glDeleteShader(kVertexShaderHandle);
    glDeleteShader(kFragmentShaderHandle);

    GLint linkSucceeded = 0;
    glGetProgramiv(programId, GL_LINK_STATUS, &linkSucceeded);
    if (!linkSucceeded)
    {
        const std::string kInfoLog = getProgramInfoLog(programId);
        MEDUSA_ERROR("Program link failed (id={}): {}", programId, kInfoLog);

        glDeleteProgram(programId);
        programId = 0;
        return false;
    }

    const std::string kInfoLog = getProgramInfoLog(programId);
    if (!kInfoLog.empty())
        MEDUSA_DEBUG("Program link succeeded with warnings (id={}): {}", programId, kInfoLog);

    MEDUSA_INFO("Shader program created (id={})", programId);
    return true;
}

void ShaderProgram::use() const
{
    glUseProgram(programId);
}

GLint ShaderProgram::loc(const char* name) const
{
    if (!name)
    {
        MEDUSA_WARN("Uniform lookup called with null name (programId={})", programId);
        return -1;
    }

    if (!programId)
    {
        MEDUSA_WARN("Uniform lookup '{}' but programId is 0", name);
        return -1;
    }

    const GLint kLocation = glGetUniformLocation(programId, name);
    if (kLocation < 0)
    {
        // Not necessarily an error: could be optimized out.
        MEDUSA_DEBUG("Uniform '{}' not found (programId={})", name, programId);
    }

    return kLocation;
}