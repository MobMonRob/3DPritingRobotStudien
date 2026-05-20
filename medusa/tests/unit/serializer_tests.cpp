// Unit tests for medusa::kronos::to_json and from_json.

#include "serializer.h"
#include "version.h"

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

using namespace medusa::kronos;

// ---------------------------------------------------------------------------
// Hilfsfunktion: minimalen, validen Toolpath erzeugen
// ---------------------------------------------------------------------------
static Toolpath make_minimal_toolpath()
{
    Toolpath tp;
    tp.metadata.slicer_version = "0.0.1";
    tp.metadata.algorithm = SlicerAlgorithm::Base;
    tp.metadata.reference_frame = "workpiece";
    tp.metadata.created_at = "2026-05-08T14:30:00Z";

    Waypoint wp;
    wp.x = 1.0;
    wp.y = 2.0;
    wp.z = 3.0;
    wp.qw = 1.0;
    wp.qx = 0.0;
    wp.qy = 0.0;
    wp.qz = 0.0;
    wp.feed_rate = 50.0;
    wp.extrusion = 2.5;
    wp.motion_type = MotionType::Print;
    wp.layer_id = 0;
    wp.segment_id = 0;
    tp.waypoints.push_back(wp);

    return tp;
}

// ---------------------------------------------------------------------------
// Tests: to_json
// ---------------------------------------------------------------------------

TEST(SerializerToJson, ContainsSchemaVersion)
{
    const auto tp = make_minimal_toolpath();
    const auto j = to_json(tp);
    ASSERT_TRUE(j.contains("schema_version"));
    EXPECT_EQ(j["schema_version"].get<std::string>(), k_schema_version);
}

TEST(SerializerToJson, ContainsAllMetadataFields)
{
    const auto tp = make_minimal_toolpath();
    const auto j = to_json(tp);
    EXPECT_TRUE(j.contains("slicer_version"));
    EXPECT_TRUE(j.contains("algorithm"));
    EXPECT_TRUE(j.contains("units"));
    EXPECT_TRUE(j.contains("reference_frame"));
    EXPECT_TRUE(j.contains("created_at"));
    EXPECT_TRUE(j.contains("waypoints_hash"));
    EXPECT_TRUE(j.contains("waypoints"));
}

TEST(SerializerToJson, WaypointsHashIsNotEmpty)
{
    const auto tp = make_minimal_toolpath();
    const auto j = to_json(tp);
    const auto hash = j["waypoints_hash"].get<std::string>();
    EXPECT_EQ(hash.size(), 16u); // FNV-1a-64: 16 Hex-Ziffern
    EXPECT_FALSE(hash.empty());
}

TEST(SerializerToJson, HashIsDeterministic)
{
    // Gleicher Input → gleicher Hash
    const auto tp = make_minimal_toolpath();
    const auto h1 = to_json(tp)["waypoints_hash"].get<std::string>();
    const auto h2 = to_json(tp)["waypoints_hash"].get<std::string>();
    EXPECT_EQ(h1, h2);
}

TEST(SerializerToJson, HashChangesWhenWaypointsChange)
{
    auto tp1 = make_minimal_toolpath();
    auto tp2 = make_minimal_toolpath();
    tp2.waypoints[0].x = 99.0; // Wegpunkt abweichend

    const auto h1 = to_json(tp1)["waypoints_hash"].get<std::string>();
    const auto h2 = to_json(tp2)["waypoints_hash"].get<std::string>();
    EXPECT_NE(h1, h2);
}

TEST(SerializerToJson, MotionTypeStringIsLowercase)
{
    auto tp = make_minimal_toolpath();
    tp.waypoints[0].motion_type = MotionType::Travel;
    const auto j = to_json(tp);
    EXPECT_EQ(j["waypoints"][0]["motion_type"].get<std::string>(), "travel");
}

TEST(SerializerToJson, AlgorithmTagIsCorrect)
{
    auto tp = make_minimal_toolpath();
    tp.metadata.algorithm = SlicerAlgorithm::Planar;
    const auto j = to_json(tp);
    EXPECT_EQ(j["algorithm"].get<std::string>(), "planar");
}

// ---------------------------------------------------------------------------
// Tests: from_json (Roundtrip)
// ---------------------------------------------------------------------------

TEST(SerializerRoundtrip, PositionSurvivesRoundtrip)
{
    auto tp = make_minimal_toolpath();
    tp.waypoints[0].x = 12.5;
    tp.waypoints[0].y = -7.3;
    tp.waypoints[0].z = 0.01;

    const auto restored = from_json(to_json(tp));
    EXPECT_DOUBLE_EQ(restored.waypoints[0].x, 12.5);
    EXPECT_DOUBLE_EQ(restored.waypoints[0].y, -7.3);
    EXPECT_DOUBLE_EQ(restored.waypoints[0].z, 0.01);
}

TEST(SerializerRoundtrip, QuaternionSurvivesRoundtrip)
{
    auto tp = make_minimal_toolpath();
    const double angle = std::numbers::pi / 4.0;
    tp.waypoints[0].qw = std::cos(angle / 2.0);
    tp.waypoints[0].qx = 0.0;
    tp.waypoints[0].qy = 0.0;
    tp.waypoints[0].qz = std::sin(angle / 2.0);

    const auto restored = from_json(to_json(tp));
    EXPECT_DOUBLE_EQ(restored.waypoints[0].qw, tp.waypoints[0].qw);
    EXPECT_DOUBLE_EQ(restored.waypoints[0].qz, tp.waypoints[0].qz);
}

TEST(SerializerRoundtrip, MetadataSurvivesRoundtrip)
{
    const auto tp = make_minimal_toolpath();
    const auto restored = from_json(to_json(tp));
    EXPECT_EQ(restored.metadata.slicer_version, tp.metadata.slicer_version);
    EXPECT_EQ(restored.metadata.reference_frame, tp.metadata.reference_frame);
    EXPECT_EQ(restored.metadata.algorithm, tp.metadata.algorithm);
}

TEST(SerializerRoundtrip, WaypointCountSurvivesRoundtrip)
{
    auto tp = make_minimal_toolpath();
    tp.waypoints.push_back(tp.waypoints[0]); // zweiten Wegpunkt hinzufügen
    const auto restored = from_json(to_json(tp));
    EXPECT_EQ(restored.waypoints.size(), 2u);
}

// ---------------------------------------------------------------------------
// Tests: from_json – Fehlerbehandlung
// ---------------------------------------------------------------------------

TEST(SerializerFromJson, ThrowsOnMissingSchemaVersion)
{
    nlohmann::json j = {{"waypoints", nlohmann::json::array()}};
    EXPECT_THROW(from_json(j), std::runtime_error);
}

TEST(SerializerFromJson, ThrowsOnIncompatibleMajorVersion)
{
    auto tp = make_minimal_toolpath();
    auto j = to_json(tp);
    j["schema_version"] = "99.0.0"; // inkompatible Major-Version
    EXPECT_THROW(from_json(j), std::runtime_error);
}

TEST(SerializerFromJson, ThrowsOnMissingWaypoints)
{
    auto tp = make_minimal_toolpath();
    auto j = to_json(tp);
    j.erase("waypoints");
    EXPECT_THROW(from_json(j), std::runtime_error);
}

TEST(SerializerFromJson, ThrowsOnUnknownMotionType)
{
    auto tp = make_minimal_toolpath();
    auto j = to_json(tp);
    j["waypoints"][0]["motion_type"] = "laser"; // ungültiger Wert
    EXPECT_THROW(from_json(j), std::runtime_error);
}
