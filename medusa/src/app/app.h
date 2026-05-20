/**
 * @file app.h
 * @brief High-level application class coordinating windowing, rendering and UI.
 */

#ifndef MEDUSA_SRC_APP_APP_H_
#define MEDUSA_SRC_APP_APP_H_

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <string>

#include "camera_controller.h"
#include "im_gui_layer.h"
#include "mesh.h"
#include "model_controller.h"
#include "axes_renderer.h"
#include "grid_renderer.h"
#include "scene_renderer.h"
#include "toolpath_renderer.h"
#include "phi_overlay_renderer.h"
#include "file_browser.h"
#include "ui_renderer.h"

#include "mesh_converter.h"
#include "triangle_mesh.h"
#include "common/i_slicer.h"
#include "planar/planar_slicer.h"
#include "base/base_slicer.h"
#include "crystal/crystal_slicer.h"
#include "crystal/crystal_params.h"
#include "common/slicer_params.h"
#include "common/slicing_pipeline.h"
#include "common/toolpath.h"

#include "exporter.h"
#include "job_metadata.h"

/**
 * @brief Main application controller.
 *
 * Responsibilities:
 * - Initialize GLFW/GLAD and create an OpenGL context
 * - Set up shaders and GPU resources
 * - Run the main loop (input, 3D rendering, ImGui rendering)
 * - Enumerate and load mesh files into a Mesh
 */
class App
{
public:
    /**
     * @brief Runs the application.
     *
     * @param argc Number of command line arguments.
     * @param argv Command line arguments.
     * @return Process exit code (0 on success).
     */
    int run(int argc, char** argv);

private:
    /** @brief GLFW window handle. */
    GLFWwindow* mWindow{nullptr};

    /** @brief Responsible for 3D scene rendering. */
    SceneRenderer mSceneRenderer;

    /** @brief Renders coordinate axes. */
    AxesRenderer mAxesRenderer;

    /** @brief Renders ground grid. */
    GridRenderer mGridRenderer;

    /** @brief The currently loaded mesh. */
    graphics::Mesh mMesh;

    /** @brief Camera controller for input handling. */
    CameraController mCameraController;

    /** @brief Responsible for UI rendering. */
    UIRenderer mUIRenderer;

    /** @brief ImGui integration layer. */
    ImGuiLayer mImGuiLayer;

    /** @brief Mesh file browser UI. */
    MeshFileBrowser mMeshFileBrowser;

    /** @brief Model controller for input handling. */
    ModelController mModelController;

    /** @brief If true, convert loaded mesh from Z-up to Y-up on import. */
    bool mZUpToYUp{true};

    /** @brief If true, render in wireframe mode. */
    bool mWireframe{false};

    /** @brief If true, render coordinate axes. */
    bool mShowAxes{true};

    /** @brief If true, render ground grid. */
    bool mShowGrid{true};

    /** @brief If true, render the toolpath overlay. */
    bool mShowToolpath{true};

    /** @brief If true, render infill lines in the toolpath. */
    bool mShowInfill{true};

    /** @brief If true, render travel (non-extrusion) moves. */
    bool mShowTravel{false};

    /** @brief If true, render TCP orientation vectors. */
    bool mShowOrientations{false};

    /** @brief If true, render mesh with transparency. */
    bool mMeshTransparent{false};

    /** @brief Time at application start (seconds, for animation timing). */
    double mStartTimeSeconds{0.0};

    /** @brief Status string for mesh loading and UI. */
    std::string mLoadStatus;

    /** @brief Currently loaded file name (for display). */
    std::string mLoadedFileName;

    /** @brief Full path of the currently loaded file (for reload on setting change). */
    std::string mLoadedFilePath;

    /** @brief Toolpath renderer. */
    ToolpathRenderer mToolpathRenderer;

    /** @brief Heatmap overlay for the Crystal harmonic scalar field Phi. */
    graphics::PhiOverlayRenderer mPhiOverlay;

    /** @brief If true, draw the Phi heatmap instead of the regular shaded mesh. */
    bool mShowPhiOverlay{false};

    /** @brief Algorithmic mesh representation for slicing. */
    geometry::TriangleMesh mTriangleMesh;

    /** @brief Currently active slicer. */
    std::unique_ptr<slicing::ISlicer> mSlicer;

    /** @brief Last pipeline result. */
    slicing::PipelineResult mPipelineResult;

    /** @brief Output directory for Kronos JSON export files. */
    std::filesystem::path mExportOutputDir;

    /** @brief Status message of the last export (empty = no export yet). */
    std::string mExportStatus;

    /** @brief Path of the last produced JSON file (empty = no export yet). */
    std::string mLastExportPath;

    /** @brief Index of the selected algorithm (0 = Planar, 1 = Base, 2 = Crystal). */
    int mSelectedAlgorithm{0};

    /** @brief Planar slicer parameters. */
    slicing::SlicerParams mPlanarParams;

    /** @brief Base slicer parameters. */
    slicing::BaseParams mBaseParams;

    /** @brief Crystal slicer parameters. */
    slicing::crystal::CrystalParams mCrystalParams;

    /**
     * @brief Initializes GLFW, creates the window and loads OpenGL function pointers.
     * @return True on success.
     */
    bool initGL();

    /**
     * @brief Shuts down OpenGL/GLFW resources.
     */
    void shutdownGL();

    /**
     * @brief Destroys the GLFW window (if any).
     */
    void cleanupWindow();

    /**
     * @brief Main loop.
     */
    void loop();

    /**
     * @brief Handles global hotkeys (e.g. Escape to close).
     */
    void handleCloseHotkeys() const;

    /**
     * @brief Updates manual yaw/pitch by mouse dragging (only when autorotation is off).
     */
    void updateManualRotationFromMouseDrag();

    /**
     * @brief Renders the 3D scene (mesh).
     */
    void renderScene3d() const;

    /**
     * @brief Renders the ImGui UI.
     */
    void renderImGui();

    /** @brief Draws the mesh file browser window. */
    void drawFilesWindow();

    /** @brief Draws the settings window (render options). */
    void drawSettingsWindow();

    /** @brief Draws the slicer settings panel. */
    void drawSlicerWindow();

    /** @brief Runs the slicing pipeline with the current settings. */
    void runSlicing();

    /** @brief Exports the last pipeline result as a Kronos JSON file. */
    void exportToolpath();

    /**
     * @brief Scans the sample data directory for mesh files.
     */
    void scanSampleFiles();

    /**
     * @brief Loads a mesh from a file path.
     * @param path File system path.
     * @return True on success.
     */
    bool loadMesh(const std::string& path);

    /**
     * @brief Resets manual rotation values.
     */
    void resetManualRotation();
};

#endif  // MEDUSA_SRC_APP_APP_H_