// Unit tests for medusa::kronos::Validator.

#include "validator.h"
#include "toolpath.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace medusa::kronos;

// ---------------------------------------------------------------------------
// Hilfsfunktion: validen Waypoint erzeugen (Einheitsquaternion, z=0.2)
// ---------------------------------------------------------------------------
static Waypoint make_valid_waypoint(std::uint32_t layer_id = 0)
{
    Waypoint wp;
    wp.x = 0.0;
    wp.y = 0.0;
    wp.z = 0.2;
    wp.qw = 1.0;
    wp.qx = 0.0;
    wp.qy = 0.0;
    wp.qz = 0.0;
    wp.feed_rate = 50.0;
    wp.extrusion = 2.5;
    wp.motion_type = MotionType::Print;
    wp.layer_id = layer_id;
    wp.segment_id = 0;
    return wp;
}

static Toolpath make_valid_toolpath()
{
    Toolpath tp;
    tp.metadata.slicer_version = "0.0.1";
    tp.metadata.reference_frame = "workpiece";
    tp.metadata.algorithm = SlicerAlgorithm::Base;
    tp.metadata.created_at = "2026-05-08T14:30:00Z";
    tp.waypoints.push_back(make_valid_waypoint(0));
    return tp;
}

// ---------------------------------------------------------------------------
// Positiv-Tests
// ---------------------------------------------------------------------------

TEST(ValidatorPositive, ValidToolpathPassesValidation)
{
    const Validator v;
    const auto result = v.validate(make_valid_toolpath());
    EXPECT_TRUE(result.is_valid());
    EXPECT_TRUE(result.errors.empty());
}

TEST(ValidatorPositive, MultipleLayers_MonotonicallyIncreasing)
{
    auto tp = make_valid_toolpath();
    tp.waypoints.push_back(make_valid_waypoint(0)); // gleiche Layer-ID erlaubt
    tp.waypoints.push_back(make_valid_waypoint(1));
    tp.waypoints.push_back(make_valid_waypoint(1));
    tp.waypoints.push_back(make_valid_waypoint(2));

    const Validator v;
    EXPECT_TRUE(v.validate(tp).is_valid());
}

// ---------------------------------------------------------------------------
// Pflichtfelder
// ---------------------------------------------------------------------------

TEST(ValidatorMandatory, EmptySlicerVersionFails)
{
    auto tp = make_valid_toolpath();
    tp.metadata.slicer_version = "";
    const Validator v;
    EXPECT_FALSE(v.validate(tp).is_valid());
}

TEST(ValidatorMandatory, EmptyReferenceFrameFails)
{
    auto tp = make_valid_toolpath();
    tp.metadata.reference_frame = "";
    const Validator v;
    EXPECT_FALSE(v.validate(tp).is_valid());
}

TEST(ValidatorMandatory, EmptyWaypointListFails)
{
    auto tp = make_valid_toolpath();
    tp.waypoints.clear();
    const Validator v;
    const auto result = v.validate(tp);
    EXPECT_FALSE(result.is_valid());
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors[0].field_path, "waypoints");
}

// ---------------------------------------------------------------------------
// Endliche Werte
// ---------------------------------------------------------------------------

TEST(ValidatorFinite, InfiniteXFails)
{
    auto tp = make_valid_toolpath();
    tp.waypoints[0].x = std::numeric_limits<double>::infinity();
    const Validator v;
    EXPECT_FALSE(v.validate(tp).is_valid());
}

TEST(ValidatorFinite, NaNZFails)
{
    auto tp = make_valid_toolpath();
    tp.waypoints[0].z = std::numeric_limits<double>::quiet_NaN();
    const Validator v;
    EXPECT_FALSE(v.validate(tp).is_valid());
}

TEST(ValidatorFinite, InfiniteFeedRateFails)
{
    auto tp = make_valid_toolpath();
    tp.waypoints[0].feed_rate = std::numeric_limits<double>::infinity();
    const Validator v;
    EXPECT_FALSE(v.validate(tp).is_valid());
}

// ---------------------------------------------------------------------------
// Quaternion-Norm
// ---------------------------------------------------------------------------

TEST(ValidatorQuaternion, UnnormalizedQuaternionFails)
{
    auto tp = make_valid_toolpath();
    tp.waypoints[0].qw = 0.5; // Norm² = 0.25, deutlich von 1 entfernt
    const Validator v;
    EXPECT_FALSE(v.validate(tp).is_valid());
}

TEST(ValidatorQuaternion, AlmostNormalizedQuaternionPasses)
{
    auto tp = make_valid_toolpath();
    // Quaternion mit minimaler numerischer Abweichung (< 1e-6)
    tp.waypoints[0].qw = 1.0 + 1e-10; // extrem nahe an 1
    tp.waypoints[0].qx = 0.0;
    tp.waypoints[0].qy = 0.0;
    tp.waypoints[0].qz = 0.0;
    // Norm² ≈ 1.0 + 2e-10, Abweichung von 1 = 2e-10 < 1e-6 → gültig
    const Validator v;
    EXPECT_TRUE(v.validate(tp).is_valid());
}

// ---------------------------------------------------------------------------
// Layer-IDs
// ---------------------------------------------------------------------------

TEST(ValidatorLayerIds, DecreasingLayerIdFails)
{
    auto tp = make_valid_toolpath();
    tp.waypoints.push_back(make_valid_waypoint(5));
    tp.waypoints.push_back(make_valid_waypoint(3)); // Rücksprung: ungültig

    const Validator v;
    const auto result = v.validate(tp);
    EXPECT_FALSE(result.is_valid());
    // Fehlerpfad soll "waypoints[2].layer_id" enthalten
    ASSERT_FALSE(result.errors.empty());
    const bool has_layer_error = std::any_of(
        result.errors.begin(), result.errors.end(),
        [](const ValidationError& e)
        {
            return e.field_path.find("layer_id") != std::string::npos;
        }
    );
    EXPECT_TRUE(has_layer_error);
}

// ---------------------------------------------------------------------------
// Fehlerstruktur
// ---------------------------------------------------------------------------

TEST(ValidatorErrors, ErrorsContainFieldPath)
{
    auto tp = make_valid_toolpath();
    tp.waypoints[0].x = std::numeric_limits<double>::quiet_NaN();
    tp.waypoints[0].y = std::numeric_limits<double>::quiet_NaN();

    const Validator v;
    const auto result = v.validate(tp);
    EXPECT_FALSE(result.is_valid());
    EXPECT_GE(result.errors.size(), 2u);
    for (const auto& err : result.errors)
    {
        EXPECT_FALSE(err.field_path.empty());
        EXPECT_FALSE(err.message.empty());
    }
}

// ---------------------------------------------------------------------------
// Erreichbarkeits-Hook
// ---------------------------------------------------------------------------

TEST(ValidatorReachability, CustomHookCanRejectWaypoint)
{
    // Einfache Test-Implementierung: lehnt alle Wegpunkte ab
    class RejectAllCheck final : public IReachabilityCheck
    {
    public:
        ValidationResult check(const Waypoint&) const override
        {
            ValidationResult r;
            r.add("position", "Außerhalb des Arbeitsraums (Test).");
            return r;
        }
    };

    auto tp = make_valid_toolpath();
    const Validator v{std::make_shared<RejectAllCheck>()};
    const auto result = v.validate(tp);
    EXPECT_FALSE(result.is_valid());
}

TEST(ValidatorReachability, DefaultHookAcceptsEverything)
{
    auto tp = make_valid_toolpath();
    const Validator v{std::make_shared<DefaultReachabilityCheck>()};
    EXPECT_TRUE(v.validate(tp).is_valid());
}
