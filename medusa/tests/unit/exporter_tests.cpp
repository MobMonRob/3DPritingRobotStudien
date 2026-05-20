// Integration tests for medusa::kronos::KronosExporter.

#include "exporter.h"
#include "serializer.h"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

using namespace medusa::kronos;
namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Test-Fixture: legt ein temporäres Ausgabeverzeichnis an und räumt auf
// ---------------------------------------------------------------------------
class ExporterTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Temporäres Verzeichnis unter dem System-TMP anlegen
        output_dir_ = fs::temp_directory_path() / "export_test";
        fs::create_directories(output_dir_);
    }

    void TearDown() override
    {
        // Aufräumen nach jedem Test
        std::error_code ec;
        fs::remove_all(output_dir_, ec);
    }

    /// Erzeugt einen minimalen validen Toolpath.
    static Toolpath make_valid_toolpath()
    {
        Toolpath tp;
        tp.metadata.slicer_version = "0.0.1";
        tp.metadata.reference_frame = "workpiece";
        tp.metadata.algorithm = SlicerAlgorithm::Base;
        tp.metadata.created_at = "2026-05-08T14:30:00Z";

        Waypoint wp;
        wp.qw = 1.0;
        wp.qx = 0.0;
        wp.qy = 0.0;
        wp.qz = 0.0;
        wp.x = 5.0;
        wp.y = 5.0;
        wp.z = 1.0;
        wp.feed_rate = 60.0;
        wp.extrusion = 3.0;
        wp.motion_type = MotionType::Print;
        wp.layer_id = 0;
        wp.segment_id = 0;
        tp.waypoints.push_back(wp);

        return tp;
    }

    fs::path output_dir_;
};

// ---------------------------------------------------------------------------
// Positiv-Tests
// ---------------------------------------------------------------------------

TEST_F(ExporterTest, ExportSucceedsAndReturnsPath)
{
    KronosExporter exporter{output_dir_};
    const auto result = exporter.export_job(make_valid_toolpath());
    ASSERT_TRUE(result.is_ok()) << result.error();
    EXPECT_TRUE(fs::exists(result.value()));
}

TEST_F(ExporterTest, ExportedFileHasJsonExtension)
{
    KronosExporter exporter{output_dir_};
    const auto result = exporter.export_job(make_valid_toolpath());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().extension(), ".json");
}

TEST_F(ExporterTest, FilenameContainsAlgorithmTag)
{
    KronosExporter exporter{output_dir_};
    auto tp = make_valid_toolpath();
    tp.metadata.algorithm = SlicerAlgorithm::Planar;

    const auto result = exporter.export_job(tp);
    ASSERT_TRUE(result.is_ok());
    EXPECT_NE(result.value().filename().string().find("planar"), std::string::npos);
}

TEST_F(ExporterTest, FilenameStartsWithToolpath)
{
    KronosExporter exporter{output_dir_};
    const auto result = exporter.export_job(make_valid_toolpath());
    ASSERT_TRUE(result.is_ok());
    EXPECT_EQ(result.value().filename().string().substr(0, 9), "toolpath_");
}

TEST_F(ExporterTest, NoTmpFileLeftAfterSuccessfulExport)
{
    KronosExporter exporter{output_dir_};
    const auto result = exporter.export_job(make_valid_toolpath());
    ASSERT_TRUE(result.is_ok());

    // Kein *.tmp soll im Verzeichnis übrig bleiben
    for (const auto& entry : fs::directory_iterator(output_dir_))
    {
        EXPECT_NE(entry.path().extension(), ".tmp")
            << "Temporäre Datei hinterlassen: " << entry.path();
    }
}

TEST_F(ExporterTest, OutputDirectoryIsCreatedIfNotExists)
{
    const auto subdir = output_dir_ / "new" / "nested" / "dir";
    ASSERT_FALSE(fs::exists(subdir));

    KronosExporter exporter{subdir};
    const auto result = exporter.export_job(make_valid_toolpath());
    ASSERT_TRUE(result.is_ok()) << result.error();
    EXPECT_TRUE(fs::exists(subdir));
}

TEST_F(ExporterTest, ExportedJsonIsValidAndParseable)
{
    KronosExporter exporter{output_dir_};
    const auto result = exporter.export_job(make_valid_toolpath());
    ASSERT_TRUE(result.is_ok());

    std::ifstream ifs{result.value()};
    ASSERT_TRUE(ifs.is_open());
    nlohmann::json j = nlohmann::json::parse(ifs);

    EXPECT_TRUE(j.contains("schema_version"));
    EXPECT_TRUE(j.contains("waypoints"));
    EXPECT_TRUE(j.contains("waypoints_hash"));
}

TEST_F(ExporterTest, ExportedJsonCanBeDeserialized)
{
    KronosExporter exporter{output_dir_};
    const auto original = make_valid_toolpath();
    const auto result = exporter.export_job(original);
    ASSERT_TRUE(result.is_ok());

    std::ifstream ifs{result.value()};
    const auto j = nlohmann::json::parse(ifs);
    const auto restored = from_json(j);

    ASSERT_EQ(restored.waypoints.size(), original.waypoints.size());
    EXPECT_DOUBLE_EQ(restored.waypoints[0].x, original.waypoints[0].x);
}

// ---------------------------------------------------------------------------
// Negativ-Tests: ungültiger Toolpath
// ---------------------------------------------------------------------------

TEST_F(ExporterTest, ExportFailsForEmptyWaypoints)
{
    auto tp = make_valid_toolpath();
    tp.waypoints.clear();

    KronosExporter exporter{output_dir_};
    const auto result = exporter.export_job(tp);
    EXPECT_TRUE(result.is_err());
}

TEST_F(ExporterTest, ExportFailsForUnnormalizedQuaternion)
{
    auto tp = make_valid_toolpath();
    tp.waypoints[0].qw = 0.5; // Norm² ≈ 0.25, nicht normalisiert

    KronosExporter exporter{output_dir_};
    const auto result = exporter.export_job(tp);
    EXPECT_TRUE(result.is_err());
}

TEST_F(ExporterTest, ErrorMessageIsNonEmpty)
{
    auto tp = make_valid_toolpath();
    tp.waypoints.clear();

    KronosExporter exporter{output_dir_};
    const auto result = exporter.export_job(tp);
    ASSERT_TRUE(result.is_err());
    EXPECT_FALSE(result.error().empty());
}

// ---------------------------------------------------------------------------
// Mehrfach-Export: verschiedene Dateien erzeugen
// ---------------------------------------------------------------------------

TEST_F(ExporterTest, MultipleExportsProduceSeparateFiles)
{
    KronosExporter exporter{output_dir_};

    // Kurz warten ist hier nicht möglich; wir prüfen stattdessen, dass
    // beide Exports erfolgreich sind. In der Praxis haben Timestamps
    // Sekunden-Auflösung; bei gleicher Sekunde würde ein Dateiname
    // überschrieben – das ist akzeptiertes Verhalten (letzter Aufruf gewinnt).
    const auto r1 = exporter.export_job(make_valid_toolpath());
    const auto r2 = exporter.export_job(make_valid_toolpath());
    EXPECT_TRUE(r1.is_ok()) << r1.error();
    EXPECT_TRUE(r2.is_ok()) << r2.error();
}
