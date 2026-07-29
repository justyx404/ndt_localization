#include "localization_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kSecond = 1000000000LL;

Eigen::Isometry3d pose(double x, double yaw_degrees = 0.0)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().x() = x;
  result.linear() =
    Eigen::AngleAxisd(
    yaw_degrees * kPi / 180.0, Eigen::Vector3d::UnitZ())
    .toRotationMatrix();
  return result;
}

std::array<double, 36> validCovariance()
{
  std::array<double, 36> covariance{};
  covariance[0] = 0.25;
  covariance[7] = 0.25;
  covariance[14] = 0.25;
  covariance[21] = 0.01;
  covariance[28] = 0.01;
  covariance[35] = 0.07;
  return covariance;
}

}  // namespace

TEST(OdometryBuffer, InterpolatesPositionAndOrientation)
{
  ndt_localization::OdometryBuffer buffer(5.0, 10);
  buffer.add(10 * kSecond, pose(0.0, 0.0));
  buffer.add(12 * kSecond, pose(4.0, 90.0));

  const auto result = buffer.lookup(11 * kSecond, 1.1);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.code, ndt_localization::DecisionCode::NONE);
  EXPECT_NEAR(result.pose.translation().x(), 2.0, 1.0e-9);
  const Eigen::AngleAxisd rotation(result.pose.rotation());
  EXPECT_NEAR(rotation.angle() * 180.0 / kPi, 45.0, 1.0e-9);
}

TEST(OdometryBuffer, IsBoundedAndRejectsUnavailableTimes)
{
  ndt_localization::OdometryBuffer buffer(2.0, 3);
  buffer.add(10 * kSecond, pose(0.0));
  buffer.add(11 * kSecond, pose(1.0));
  buffer.add(12 * kSecond, pose(2.0));
  buffer.add(13 * kSecond, pose(3.0));

  EXPECT_EQ(buffer.size(), 3u);
  EXPECT_EQ(
    buffer.lookup(10 * kSecond, 1.0).code,
    ndt_localization::DecisionCode::ODOMETRY_TOO_OLD);
  EXPECT_EQ(
    buffer.lookup(14 * kSecond, 1.0).code,
    ndt_localization::DecisionCode::ODOMETRY_TOO_NEW);
  EXPECT_EQ(
    buffer.lookup(11500 * kSecond / 1000, 0.4).code,
    ndt_localization::DecisionCode::ODOMETRY_INTERPOLATION_GAP);
}

TEST(InitialPoseValidation, AcceptsFiniteBoundedCovariance)
{
  ndt_localization::InitialPoseValidationLimits limits;
  const auto result = ndt_localization::validateInitialPoseData(
    Eigen::Vector3d(1.0, 2.0, 3.0),
    Eigen::Quaterniond::Identity(), validCovariance(), limits);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.code, ndt_localization::DecisionCode::NONE);
}

TEST(TimestampValidation, RejectsInvalidStaleAndFutureInputs)
{
  const auto validate =
    [](std::int64_t timestamp_ns)
    {
      return ndt_localization::validateTimestampNanoseconds(
        timestamp_ns, 10 * kSecond, 1.0, 0.1,
        ndt_localization::DecisionCode::INITIAL_POSE_STAMP_INVALID,
        ndt_localization::DecisionCode::INITIAL_POSE_STALE,
        ndt_localization::DecisionCode::INITIAL_POSE_FUTURE);
    };

  EXPECT_EQ(
    validate(0).code,
    ndt_localization::DecisionCode::INITIAL_POSE_STAMP_INVALID);
  EXPECT_EQ(
    validate(8 * kSecond).code,
    ndt_localization::DecisionCode::INITIAL_POSE_STALE);
  EXPECT_EQ(
    validate(11 * kSecond).code,
    ndt_localization::DecisionCode::INITIAL_POSE_FUTURE);
  EXPECT_TRUE(validate(9500 * kSecond / 1000).valid);
}

TEST(TimestampValidation, InitializationRequiresLaterScans)
{
  EXPECT_EQ(
    ndt_localization::validateInitializationScanTimestamp(
      9 * kSecond, 10 * kSecond).code,
    ndt_localization::DecisionCode::SCAN_PRECEDES_INITIALIZATION);
  EXPECT_EQ(
    ndt_localization::validateInitializationScanTimestamp(
      10 * kSecond, 10 * kSecond).code,
    ndt_localization::DecisionCode::SCAN_PRECEDES_INITIALIZATION);
  EXPECT_TRUE(
    ndt_localization::validateInitializationScanTimestamp(
      11 * kSecond, 10 * kSecond).valid);
}

TEST(WorkloadBounds, DetectsDeadlineAtBudget)
{
  EXPECT_FALSE(
    ndt_localization::deadlineExpired(
      10 * kSecond, 10 * kSecond + 79999999LL, 80.0));
  EXPECT_TRUE(
    ndt_localization::deadlineExpired(
      10 * kSecond, 10 * kSecond + 80000000LL, 80.0));
  EXPECT_TRUE(
    ndt_localization::deadlineExpired(
      10 * kSecond, 9 * kSecond, 80.0));
  EXPECT_TRUE(
    ndt_localization::deadlineExpired(
      -1, 10 * kSecond, 80.0));
}

TEST(WorkloadBounds, SelectsDeterministicEvenlySpacedIndices)
{
  EXPECT_EQ(
    ndt_localization::deterministicSampleIndices(0, 5),
    (std::vector<std::size_t>{}));
  EXPECT_EQ(
    ndt_localization::deterministicSampleIndices(5, 0),
    (std::vector<std::size_t>{}));
  EXPECT_EQ(
    ndt_localization::deterministicSampleIndices(3, 5),
    (std::vector<std::size_t>{0, 1, 2}));
  EXPECT_EQ(
    ndt_localization::deterministicSampleIndices(10, 4),
    (std::vector<std::size_t>{0, 2, 5, 7}));
}

TEST(InitializationSearch, DerivesBoundedCovarianceAwareEnvelope)
{
  const auto bounds = ndt_localization::initializationSearchBounds(
    2.0, 30.0, 2.5, 1.0, 10.0, 15.0, 180.0);
  EXPECT_DOUBLE_EQ(bounds.translation_span_m, 5.0);
  EXPECT_DOUBLE_EQ(bounds.yaw_span_deg, 75.0);

  const auto capped = ndt_localization::initializationSearchBounds(
    10.0, 180.0, 2.5, 1.0, 10.0, 15.0, 180.0);
  EXPECT_DOUBLE_EQ(capped.translation_span_m, 10.0);
  EXPECT_DOUBLE_EQ(capped.yaw_span_deg, 180.0);
}

TEST(InitializationSearch, UsesPriorYawAndGravityAlignedTilt)
{
  Eigen::Isometry3d prior = Eigen::Isometry3d::Identity();
  prior.translation() = Eigen::Vector3d(3.0, -2.0, 1.0);
  prior.linear() =
    (Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX())).toRotationMatrix();

  Eigen::Isometry3d odometry = Eigen::Isometry3d::Identity();
  odometry.linear() =
    (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX())).toRotationMatrix();

  const Eigen::Isometry3d constrained =
    ndt_localization::gravityConstrainedPose(prior, odometry);
  const Eigen::Matrix3d expected_rotation =
    Eigen::AngleAxisd(
    0.9, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    odometry.rotation();
  EXPECT_TRUE(constrained.translation().isApprox(prior.translation()));
  EXPECT_TRUE(constrained.rotation().isApprox(expected_rotation, 1.0e-12));
  EXPECT_FALSE(constrained.rotation().isApprox(prior.rotation(), 1.0e-3));
}

TEST(InitializationSearch, GeneratesDeterministicCombinedHypotheses)
{
  ndt_localization::InitializationSearchBounds bounds;
  bounds.translation_span_m = 4.0;
  bounds.yaw_span_deg = 60.0;
  const auto hypotheses =
    ndt_localization::deterministicHypothesisOffsets(bounds, 65);
  ASSERT_EQ(hypotheses.size(), 65u);
  EXPECT_DOUBLE_EQ(hypotheses.front().x_m, 0.0);
  EXPECT_DOUBLE_EQ(hypotheses.front().yaw_deg, 0.0);

  const auto combined = std::find_if(
    hypotheses.begin(), hypotheses.end(),
    [](const ndt_localization::HypothesisOffset & hypothesis)
    {
      return hypothesis.x_m == 2.0 &&
             hypothesis.y_m == 0.0 &&
             hypothesis.yaw_deg == 30.0;
    });
  EXPECT_NE(combined, hypotheses.end());
}

TEST(InitializationSearch, RejectsDistinctAmbiguousRunnerUp)
{
  std::vector<ndt_localization::ScoredPose> candidates(3);
  candidates[0].pose = pose(0.0);
  candidates[0].score = 0.10;
  candidates[1].pose = pose(0.1);
  candidates[1].score = 0.105;
  candidates[2].pose = pose(2.0);
  candidates[2].score = 0.30;

  auto selection = ndt_localization::selectBestHypothesis(
    candidates, 0.5, 0.01, 0.5, 5.0);
  EXPECT_TRUE(selection.valid);
  EXPECT_FALSE(selection.ambiguous);
  EXPECT_EQ(selection.best_index, 0u);
  EXPECT_EQ(selection.second_index, 2u);
  EXPECT_NEAR(selection.score_margin, 0.20, 1.0e-12);

  candidates[2].score = 0.105;
  selection = ndt_localization::selectBestHypothesis(
    candidates, 0.5, 0.01, 0.5, 5.0);
  EXPECT_FALSE(selection.valid);
  EXPECT_TRUE(selection.ambiguous);
}

TEST(InitialPoseValidation, RejectsMalformedAndAmbiguousInputs)
{
  ndt_localization::InitialPoseValidationLimits limits;
  auto covariance = validCovariance();
  Eigen::Quaterniond invalid_quaternion(0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(
    ndt_localization::validateInitialPoseData(
      Eigen::Vector3d::Zero(), invalid_quaternion,
      covariance, limits).code,
    ndt_localization::DecisionCode::INITIAL_POSE_POSE_INVALID);

  covariance = validCovariance();
  covariance[1] = 1.0;
  EXPECT_EQ(
    ndt_localization::validateInitialPoseData(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
      covariance, limits).code,
    ndt_localization::DecisionCode::INITIAL_POSE_COVARIANCE_INVALID);

  covariance = validCovariance();
  covariance[0] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    ndt_localization::validateInitialPoseData(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
      covariance, limits).code,
    ndt_localization::DecisionCode::INITIAL_POSE_COVARIANCE_INVALID);

  covariance = validCovariance();
  covariance[0] = 101.0;
  EXPECT_EQ(
    ndt_localization::validateInitialPoseData(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
      covariance, limits).code,
    ndt_localization::DecisionCode::INITIAL_POSE_COVARIANCE_AMBIGUOUS);
}

TEST(TransformValidation, RejectsNonFiniteNonRigidAndLargeJumps)
{
  Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
  candidate.translation().x() =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    ndt_localization::validateTransformCandidate(
      Eigen::Isometry3d::Identity(), candidate, 1.0, 20.0).code,
    ndt_localization::DecisionCode::RESULT_NON_FINITE);

  candidate = Eigen::Isometry3d::Identity();
  candidate.linear()(0, 0) = 2.0;
  EXPECT_EQ(
    ndt_localization::validateTransformCandidate(
      Eigen::Isometry3d::Identity(), candidate, 1.0, 20.0).code,
    ndt_localization::DecisionCode::RESULT_NOT_RIGID);

  EXPECT_EQ(
    ndt_localization::validateTransformCandidate(
      Eigen::Isometry3d::Identity(), pose(2.0), 1.0, 20.0).code,
    ndt_localization::DecisionCode::RESULT_TRANSLATION_JUMP);
  EXPECT_EQ(
    ndt_localization::validateTransformCandidate(
      Eigen::Isometry3d::Identity(), pose(0.0, 30.0), 1.0, 20.0).code,
    ndt_localization::DecisionCode::RESULT_ROTATION_JUMP);
}

TEST(LocalizationStateMachine, RequiresConsecutiveConfirmation)
{
  ndt_localization::LocalizationStateMachine machine(3);
  machine.beginInitialization(pose(1.0));

  EXPECT_EQ(
    machine.state(), ndt_localization::LocalizationState::INITIALIZING);
  EXPECT_FALSE(machine.hasValidCorrection());

  auto first = machine.observeInitializationCorrection(
    pose(1.1), 0.5, 10.0);
  EXPECT_TRUE(first.accepted);
  EXPECT_FALSE(first.confirmed);
  EXPECT_EQ(first.confirmation_count, 1u);
  EXPECT_FALSE(machine.hasValidCorrection());

  const auto rejected = machine.observeInitializationCorrection(
    pose(2.0), 0.5, 10.0);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(machine.confirmationCount(), 0u);
  EXPECT_FALSE(machine.hasValidCorrection());

  machine.observeInitializationCorrection(pose(1.1), 0.5, 10.0);
  machine.observeInitializationCorrection(pose(1.2), 0.5, 10.0);
  const auto confirmed = machine.observeInitializationCorrection(
    pose(1.3), 0.5, 10.0);
  EXPECT_TRUE(confirmed.confirmed);
  EXPECT_TRUE(machine.hasValidCorrection());
  EXPECT_EQ(
    machine.state(), ndt_localization::LocalizationState::TRACKING);
  Eigen::Isometry3d published_correction = Eigen::Isometry3d::Identity();
  ASSERT_TRUE(machine.getValidCorrection(&published_correction));
  EXPECT_NEAR(
    published_correction.translation().x(), 1.3, 1.0e-9);
}

TEST(LocalizationStateMachine, RetainsLastCorrectionWhenTrackingIsLost)
{
  ndt_localization::LocalizationStateMachine machine(1);
  machine.beginInitialization(pose(1.0));
  ASSERT_TRUE(
    machine.observeInitializationCorrection(
      pose(1.0), 0.5, 10.0).confirmed);
  ASSERT_TRUE(machine.applyTrackingCorrection(pose(1.2)));

  EXPECT_FALSE(machine.recordTrackingRejection(2));
  EXPECT_TRUE(machine.recordTrackingRejection(2));
  EXPECT_EQ(machine.state(), ndt_localization::LocalizationState::LOST);
  EXPECT_TRUE(machine.hasValidCorrection());
  EXPECT_NEAR(machine.correction().translation().x(), 1.2, 1.0e-9);
  EXPECT_FALSE(machine.applyTrackingCorrection(pose(2.0)));
}

TEST(LocalizationStateMachine, NewInitializationSuspendsPublishedCorrection)
{
  ndt_localization::LocalizationStateMachine machine(1);
  machine.beginInitialization(pose(1.0));
  ASSERT_TRUE(
    machine.observeInitializationCorrection(
      pose(1.0), 0.5, 10.0).confirmed);
  ASSERT_TRUE(machine.hasValidCorrection());

  machine.beginInitialization(pose(4.0));

  EXPECT_EQ(
    machine.state(), ndt_localization::LocalizationState::INITIALIZING);
  EXPECT_FALSE(machine.hasValidCorrection());
  EXPECT_FALSE(machine.getValidCorrection(nullptr));
}

TEST(LocalizationStateMachine, RelocalizationRetainsFallbackUntilConfirmed)
{
  ndt_localization::LocalizationStateMachine machine(2);
  machine.beginInitialization(pose(1.0));
  machine.observeInitializationCorrection(pose(1.0), 0.5, 10.0);
  ASSERT_TRUE(
    machine.observeInitializationCorrection(
      pose(1.0), 0.5, 10.0).confirmed);

  machine.beginRelocalization(pose(2.0));
  EXPECT_EQ(
    machine.state(), ndt_localization::LocalizationState::RELOCALIZING);
  ASSERT_TRUE(machine.hasValidCorrection());
  EXPECT_NEAR(machine.correction().translation().x(), 1.0, 1.0e-9);
  ASSERT_TRUE(machine.setInitializationCandidate(pose(2.0)));

  const auto pending = machine.observeInitializationCorrection(
    pose(2.1), 0.5, 10.0);
  EXPECT_EQ(
    pending.code,
    ndt_localization::DecisionCode::RELOCALIZATION_CONFIRMATION_PENDING);
  EXPECT_NEAR(machine.correction().translation().x(), 1.0, 1.0e-9);

  const auto confirmed = machine.observeInitializationCorrection(
    pose(2.2), 0.5, 10.0);
  EXPECT_TRUE(confirmed.confirmed);
  EXPECT_EQ(
    confirmed.code,
    ndt_localization::DecisionCode::RELOCALIZATION_CONFIRMED);
  EXPECT_EQ(machine.state(), ndt_localization::LocalizationState::TRACKING);
  EXPECT_NEAR(machine.correction().translation().x(), 2.2, 1.0e-9);
}

TEST(LocalizationStateMachine, FailedRelocalizationKeepsLastCorrection)
{
  ndt_localization::LocalizationStateMachine machine(1);
  machine.beginInitialization(pose(1.0));
  ASSERT_TRUE(
    machine.observeInitializationCorrection(
      pose(1.0), 0.5, 10.0).confirmed);
  machine.beginRelocalization(pose(3.0));

  EXPECT_EQ(
    machine.failInitializationAttempt(),
    ndt_localization::LocalizationState::LOST);
  EXPECT_TRUE(machine.hasValidCorrection());
  EXPECT_NEAR(machine.correction().translation().x(), 1.0, 1.0e-9);
}
