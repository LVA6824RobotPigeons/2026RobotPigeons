// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$0;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$1;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$2;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$3;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoContext;
import frc.robot.commands.auto.AutoOrchestrator;
import frc.robot.commands.auto.AutoPhase;
import frc.robot.commands.auto.AutoPlannerClient;
import frc.robot.commands.auto.AutoSafetyGate;
import frc.robot.commands.auto.AutoSafetyGate.Decision;
import frc.robot.commands.auto.AutoSafetyGate.ValidationResult;
import frc.robot.commands.auto.AutoStatus;
import frc.robot.commands.auto.CommandAutoPhase;
import frc.robot.commands.auto.FullCycleCommand;
import frc.robot.commands.auto.FuelDetector;
import frc.robot.commands.auto.LocalAutoPlannerClient;
import frc.robot.commands.auto.PlannerExecutionMode;
import frc.robot.commands.auto.ShiftGameData;
import frc.robot.commands.auto.bcnp.BcnpSchemaHash;
import frc.robot.commands.auto.bcnp.BcnpTcpPlannerClient;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class AutoRoutines {
    private static final double kFieldLengthMeters = 16.541;
    private static final double kSeedAndDepartTimeoutSeconds = 4.5;
    private static final double kCollectDepotTimeoutSeconds = 4.0;
    private static final double kAdvanceDepotTimeoutSeconds = 3.0;
    private static final double kTransitAndPreSpinTimeoutSeconds = 4.0;
    private static final double kAimAndShootTimeoutSeconds = 4.0;
    private static final double kSafeExitTimeoutSeconds = 1.2;
    private static final double kAdaptiveCollectionMaxElapsedSeconds = 6.5;
    private static final double kAdaptiveTrajectoryTransitMaxElapsedSeconds = 9.5;
    private static final LocalAutoProfile kLeftProfile = new LocalAutoProfile(
            "M1 - L Max Fuel",
            1,
            2600,
            0.32,
            101,
            LocalAutoStyle.MAX_FUEL);
    private static final LocalAutoProfile kCenterProfile = new LocalAutoProfile(
            "M1 - C Max Fuel",
            2,
            2550,
            0.31,
            102,
            LocalAutoStyle.FAST_CYCLE_SCORE);
    private static final LocalAutoProfile kRightProfile = new LocalAutoProfile(
            "M1 - R Fast Score Exit",
            3,
            2450,
            0.30,
            103,
            LocalAutoStyle.SAFE_SCORE_EXIT);
    private static final LocalAutoProfile kSafeScoreProfile = new LocalAutoProfile(
            "M2 - Safe Score Exit",
            11,
            2500,
            0.30,
            201,
            LocalAutoStyle.SAFE_SCORE_EXIT);
    private static final LocalAutoProfile kFastCycleProfile = new LocalAutoProfile(
            "M2 - Fast Cycle Score",
            12,
            2620,
            0.31,
            202,
            LocalAutoStyle.FAST_CYCLE_SCORE);
    private static final LocalAutoProfile kConservativeMobilityProfile = new LocalAutoProfile(
            "M2 - Conservative Mobility",
            13,
            0,
            0,
            203,
            LocalAutoStyle.CONSERVATIVE_MOBILITY);

    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;

    private final SubsystemCommands subsystemCommands;
    private final AutoSafetyGate safetyGate = new AutoSafetyGate();
    private final AutoPlannerClient plannerClient;
    private final PlannerExecutionMode configuredPlannerMode;
    private final AutoStatus autoStatus = new AutoStatus();
    private final AutoOrchestrator autoOrchestrator = new AutoOrchestrator(autoStatus);
    private final AutoContext autoContext;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;
    private volatile String dynamicSelectionName = "none";

    public AutoRoutines(
            Swerve swerve,
            Intake intake,
            Floor floor,
            Feeder feeder,
            Shooter shooter,
            Hood hood,
            Hanger hanger,
            Limelight limelight) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
        this.configuredPlannerMode = Constants.Autonomous.kUseBcnpPlanner
                ? Constants.Autonomous.kPlannerExecutionMode
                : PlannerExecutionMode.LOCAL_ONLY;
        final int schemaHash = BcnpSchemaHash.loadOrFallback(
                Constants.Autonomous.kBcnpSchemaDeployPath,
                Constants.Autonomous.kBcnpSchemaHashFallback);
        this.plannerClient = Constants.Autonomous.kUseBcnpPlanner
                ? new BcnpTcpPlannerClient(
                        Constants.Autonomous.kBcnpHost,
                        Constants.Autonomous.kBcnpPort,
                        schemaHash,
                        Constants.Autonomous.kBcnpConnectRetryMs,
                        Constants.Autonomous.kBcnpHeartbeatPeriodMs,
                        Constants.Autonomous.kBcnpHeartbeatTimeoutMs)
                : new LocalAutoPlannerClient();
        this.autoContext = new AutoContext(
                () -> swerve.getState().Pose,
                DriverStation::getAlliance,
                hanger::isHomed,
                plannerClient::isHealthy);
    }

    public AutoPlannerClient getPlannerClient() {
        return plannerClient;
    }

    public Optional<Command> buildFullCycleCommand() {
        if (!(plannerClient instanceof BcnpTcpPlannerClient bcnpPlanner)) {
            return Optional.empty();
        }
        final FuelDetector fuelDetector = new FuelDetector(intake);
        return Optional.of(
                Commands.sequence(
                        Commands.runOnce(() -> {
                            final DriverStation.Alliance alliance = DriverStation.getAlliance()
                                    .orElse(DriverStation.Alliance.Blue);
                            bcnpPlanner.setPlanRequestContext(kFastCycleProfile.profileId(), autoContext.robotPose(),
                                    alliance);
                        }),
                        new FullCycleCommand(
                                swerve,
                                intake,
                                shooter,
                                hood,
                                feeder,
                                floor,
                                fuelDetector,
                                bcnpPlanner,
                                autoContext::robotPose))
                        .withName("FullCycleCommand"));
    }

    public void configure() {
        // Keep the legacy routine available while M0/M1 routines are stabilized.
        autoChooser.addRoutine("LEGACY - Outpost and Depot", this::outpostAndDepotLegacyRoutine);
        autoChooser.addRoutine(kSafeScoreProfile.chooserName(), () -> phaseDrivenRoutine(kSafeScoreProfile));
        autoChooser.addRoutine(kFastCycleProfile.chooserName(), () -> phaseDrivenRoutine(kFastCycleProfile));
        autoChooser.addRoutine(
                kConservativeMobilityProfile.chooserName(),
                () -> phaseDrivenRoutine(kConservativeMobilityProfile));
        autoChooser.addRoutine("M2 - Dynamic Classical Selector", this::dynamicClassicalRoutine);
        autoChooser.addRoutine("M3 - Shadow Adaptive Classical", this::shadowAdaptiveClassicalRoutine);
        autoChooser.addRoutine(kLeftProfile.chooserName(), () -> phaseDrivenRoutine(kLeftProfile));
        autoChooser.addRoutine(kCenterProfile.chooserName(), () -> phaseDrivenRoutine(kCenterProfile));
        autoChooser.addRoutine(kRightProfile.chooserName(), () -> phaseDrivenRoutine(kRightProfile));
        if (plannerClient instanceof BcnpTcpPlannerClient) {
            autoChooser.addRoutine("M4 - BCNP Full Cycle", this::fullCycleRoutine);
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Publish orchestrator health/status continuously while autonomous is active.
        RobotModeTriggers.autonomous().whileTrue(Commands.run(this::publishStatus));
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine fullCycleRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("M4 - BCNP Full Cycle");
        routine.active().onTrue(buildFullCycleCommand().orElse(Commands.none()));
        return routine;
    }

    private void publishStatus() {
        if (plannerClient instanceof BcnpTcpPlannerClient bcnpPlanner) {
            final DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            bcnpPlanner.updatePoseContext(autoContext.robotPose(), alliance);
        }
        plannerClient.periodic();
        final PlannerExecutionMode effectiveMode = effectivePlannerMode();
        final Optional<AutoPlannerClient.RemotePlan> remotePlan = plannerClient.latestPlan();
        final ShadowAgreement shadowAgreement = computeShadowAgreement(remotePlan);

        SmartDashboard.putString("Auto/Status/Routine", autoStatus.routineId());
        SmartDashboard.putString("Auto/Status/Phase", autoStatus.phaseId());
        SmartDashboard.putString("Auto/Status/Link", autoStatus.linkState().name());
        SmartDashboard.putString("Auto/Status/PlannerMode", autoStatus.plannerMode().name());
        SmartDashboard.putBoolean("Auto/Status/Fallback", autoStatus.fallbackActive());
        SmartDashboard.putBoolean("Auto/Status/Completed", autoStatus.completed());
        SmartDashboard.putBoolean("Auto/Status/Aborted", autoStatus.aborted());
        SmartDashboard.putString("Auto/Status/LastGateCode", autoStatus.lastGateCode());
        SmartDashboard.putString("Auto/Status/LastGateDetail", autoStatus.lastGateDetail());
        SmartDashboard.putString("Auto/Planner/ConfiguredMode", configuredPlannerMode.name());
        SmartDashboard.putString("Auto/Planner/EffectiveMode", effectiveMode.name());
        SmartDashboard.putString("Auto/Planner/Fault", plannerClient.lastFault());
        SmartDashboard.putNumber("Auto/Planner/HeartbeatAgeSec", plannerClient.heartbeatAgeSeconds());
        SmartDashboard.putNumber("Auto/Planner/PlanId", remotePlan.map(plan -> (double) plan.planId()).orElse(-1.0));
        SmartDashboard.putNumber("Auto/Planner/PlanPhaseCount",
                remotePlan.map(plan -> (double) plan.phaseCount()).orElse(-1.0));
        SmartDashboard.putNumber("Auto/Planner/PlanProfileId",
                remotePlan.map(plan -> (double) plan.profileId()).orElse(-1.0));
        SmartDashboard.putNumber("Auto/Planner/ObjectiveId",
                remotePlan.map(plan -> (double) plan.objectiveId()).orElse(-1.0));
        SmartDashboard.putNumber(
                "Auto/Planner/GlobalConfidencePermille",
                remotePlan.map(plan -> (double) plan.globalConfidencePermille()).orElse(-1.0));
        SmartDashboard.putString(
                "Auto/Planner/PolicySource",
                remotePlan.map(plan -> policySourceName(plan.policySource())).orElse("none"));
        SmartDashboard.putString("Auto/Planner/DynamicSelection", dynamicSelectionName);
        SmartDashboard.putBoolean(
                "Auto/Planner/PlanFresh",
                remotePlan.map(plan -> plan.isFresh(System.currentTimeMillis(), Constants.Autonomous.kBcnpPlanFreshMs))
                        .orElse(false));
        SmartDashboard.putBoolean("Auto/Planner/ShadowAgreement", shadowAgreement.agrees());
        SmartDashboard.putString("Auto/Planner/ShadowAgreementReason", shadowAgreement.reason());
        SmartDashboard.putString(
                "Auto/GameData/Shift1InactiveHub",
                ShiftGameData.parse(DriverStation.getGameSpecificMessage()).name());
    }

    private AutoRoutine dynamicClassicalRoutine() {
        final LocalAutoProfile selected = selectDynamicClassicalProfile();
        dynamicSelectionName = selected.chooserName();
        final LocalAutoProfile dynamicWrappedProfile = new LocalAutoProfile(
                "M2 - Dynamic -> " + selected.chooserName(),
                selected.profileId(),
                selected.shooterRpm(),
                selected.hoodPosition(),
                selected.objectiveId(),
                LocalAutoStyle.ADAPTIVE_CLASSICAL);
        return phaseDrivenRoutine(dynamicWrappedProfile);
    }

    private AutoRoutine shadowAdaptiveClassicalRoutine() {
        final LocalAutoProfile selected = selectShadowAdaptiveProfile();
        dynamicSelectionName = selected.chooserName();
        final LocalAutoProfile adaptiveWrappedProfile = new LocalAutoProfile(
                "M3 - Shadow Adaptive -> " + selected.chooserName(),
                selected.profileId(),
                selected.shooterRpm(),
                selected.hoodPosition(),
                selected.objectiveId(),
                LocalAutoStyle.ADAPTIVE_CLASSICAL);
        return phaseDrivenRoutine(adaptiveWrappedProfile);
    }

    private LocalAutoProfile selectDynamicClassicalProfile() {
        if (!autoContext.isHangerHomed()) {
            return kConservativeMobilityProfile;
        }

        final Optional<AutoPlannerClient.RemotePlan> remotePlan = plannerClient.latestPlan();
        if (remotePlan.isPresent()) {
            final AutoPlannerClient.RemotePlan plan = remotePlan.get();
            final boolean planUsable = plan.isFresh(System.currentTimeMillis(), Constants.Autonomous.kBcnpPlanFreshMs)
                    && plan.globalConfidencePermille() >= Constants.Autonomous.kActivePlanMinConfidencePermille
                    && (plan.policySource() == AutoPlannerClient.POLICY_SOURCE_LOCAL
                            || plan.policySource() == AutoPlannerClient.POLICY_SOURCE_CLASSICAL);
            if (planUsable) {
                final LocalAutoProfile mappedProfile = profileForId(plan.profileId());
                if (mappedProfile != null) {
                    return mappedProfile;
                }
                final LocalAutoProfile objectiveMappedProfile = profileForObjective(plan.objectiveId());
                if (objectiveMappedProfile != null) {
                    return objectiveMappedProfile;
                }
            }
        }

        final Optional<DriverStation.Alliance> alliance = autoContext.alliance();
        if (alliance.isEmpty()) {
            return kSafeScoreProfile;
        }

        final ShiftGameData shiftData = ShiftGameData.parse(DriverStation.getGameSpecificMessage());
        if (isOwnShiftInactive(shiftData, alliance.get())) {
            return kFastCycleProfile;
        }
        return kSafeScoreProfile;
    }

    private LocalAutoProfile selectShadowAdaptiveProfile() {
        final Optional<AutoPlannerClient.RemotePlan> remotePlan = plannerClient.latestPlan();
        if (remotePlan.isPresent()) {
            final AutoPlannerClient.RemotePlan plan = remotePlan.get();
            final boolean shadowHintUsable = plan.isFresh(System.currentTimeMillis(),
                    Constants.Autonomous.kBcnpPlanFreshMs)
                    && plan.globalConfidencePermille() >= Constants.Autonomous.kActivePlanMinConfidencePermille
                    && (plan.policySource() == AutoPlannerClient.POLICY_SOURCE_LOCAL
                            || plan.policySource() == AutoPlannerClient.POLICY_SOURCE_CLASSICAL
                            || plan.policySource() == AutoPlannerClient.POLICY_SOURCE_LEARNED_SHADOW);
            if (shadowHintUsable) {
                final LocalAutoProfile mappedProfile = profileForObjective(plan.objectiveId());
                if (mappedProfile != null) {
                    return mappedProfile;
                }
                final LocalAutoProfile profileMapped = profileForId(plan.profileId());
                if (profileMapped != null) {
                    return profileMapped;
                }
            }
        }
        return selectDynamicClassicalProfile();
    }

    private boolean isOwnShiftInactive(ShiftGameData shiftData, DriverStation.Alliance alliance) {
        return switch (alliance) {
            case Blue -> shiftData == ShiftGameData.BLUE_INACTIVE_SHIFT1;
            case Red -> shiftData == ShiftGameData.RED_INACTIVE_SHIFT1;
        };
    }

    private LocalAutoProfile profileForId(int profileId) {
        if (profileId == kLeftProfile.profileId()) {
            return kLeftProfile;
        }
        if (profileId == kCenterProfile.profileId()) {
            return kCenterProfile;
        }
        if (profileId == kRightProfile.profileId()) {
            return kRightProfile;
        }
        if (profileId == kSafeScoreProfile.profileId()) {
            return kSafeScoreProfile;
        }
        if (profileId == kFastCycleProfile.profileId()) {
            return kFastCycleProfile;
        }
        if (profileId == kConservativeMobilityProfile.profileId()) {
            return kConservativeMobilityProfile;
        }
        return null;
    }

    private LocalAutoProfile profileForObjective(int objectiveId) {
        if (objectiveId == kLeftProfile.objectiveId()) {
            return kLeftProfile;
        }
        if (objectiveId == kCenterProfile.objectiveId()) {
            return kCenterProfile;
        }
        if (objectiveId == kRightProfile.objectiveId()) {
            return kRightProfile;
        }
        if (objectiveId == kSafeScoreProfile.objectiveId()) {
            return kSafeScoreProfile;
        }
        if (objectiveId == kFastCycleProfile.objectiveId()) {
            return kFastCycleProfile;
        }
        if (objectiveId == kConservativeMobilityProfile.objectiveId()) {
            return kConservativeMobilityProfile;
        }
        return null;
    }

    private String policySourceName(int policySource) {
        return switch (policySource) {
            case AutoPlannerClient.POLICY_SOURCE_LOCAL -> "LOCAL";
            case AutoPlannerClient.POLICY_SOURCE_CLASSICAL -> "CLASSICAL";
            case AutoPlannerClient.POLICY_SOURCE_LEARNED_SHADOW -> "LEARNED_SHADOW";
            case AutoPlannerClient.POLICY_SOURCE_LEARNED_ACTIVE -> "LEARNED_ACTIVE";
            default -> "UNKNOWN_" + policySource;
        };
    }

    private AutoRoutine phaseDrivenRoutine(LocalAutoProfile profile) {
        if (plannerClient instanceof BcnpTcpPlannerClient bcnpPlanner) {
            final DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            bcnpPlanner.setPlanRequestContext(profile.profileId(), autoContext.robotPose(), alliance);
        }
        final AutoRoutine routine = autoFactory.newRoutine(profile.chooserName());
        final AutoTrajectory startToOutpost = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);

        final AtomicBoolean reachedOutpost = new AtomicBoolean(false);
        final AtomicBoolean reachedDepot = new AtomicBoolean(false);
        final AtomicBoolean reachedShootingPose = new AtomicBoolean(false);
        final AtomicBoolean shotComplete = new AtomicBoolean(false);
        final AtomicBoolean exitComplete = new AtomicBoolean(false);
        final AtomicReference<Double> autoStartTimestampSeconds = new AtomicReference<>(null);
        final PlannerExecutionMode routineMode = configuredPlannerMode;
        final BooleanSupplier forcedFallbackCondition = () -> shouldForceFallbackToLocal(profile, routineMode);

        final AtomicReference<ValidationResult> shotValidation = new AtomicReference<>(
                ValidationResult.denyRetryable("SHOT_NOT_EVALUATED", "Shot gate has not run yet."));
        final AtomicReference<ValidationResult> traversalValidation = new AtomicReference<>(
                ValidationResult.denyRetryable("TRAVERSAL_NOT_EVALUATED", "Traversal gate has not run yet."));
        final AtomicReference<ValidationResult> collectionValidation = new AtomicReference<>(
                ValidationResult.denyRetryable("COLLECTION_NOT_EVALUATED", "Collection gate has not run yet."));

        startToOutpost.done().onTrue(Commands.runOnce(() -> reachedOutpost.set(true)));
        outpostToDepot.done().onTrue(Commands.runOnce(() -> reachedDepot.set(true)));
        depotToShootingPose.done().onTrue(Commands.runOnce(() -> reachedShootingPose.set(true)));

        final List<AutoPhase> phases = new ArrayList<>();
        phases.add(buildSeedAndDepartPhase(startToOutpost, traversalValidation, reachedOutpost, exitComplete));
        switch (profile.style()) {
            case MAX_FUEL -> {
                phases.add(
                        buildCollectDepotFuelPhase(outpostToDepot, collectionValidation, reachedDepot, exitComplete));
                phases.add(
                        buildTransitAndPreSpinPhase(profile, depotToShootingPose, reachedShootingPose, exitComplete));
                phases.add(buildAimAndShootPhase(profile, shotComplete, shotValidation, exitComplete));
                phases.add(buildExitPhase(exitComplete));
            }
            case FAST_CYCLE_SCORE -> {
                phases.add(
                        buildCollectDepotFuelPhase(outpostToDepot, collectionValidation, reachedDepot, exitComplete));
                phases.add(
                        buildTransitAndPreSpinPhase(profile, depotToShootingPose, reachedShootingPose, exitComplete));
                phases.add(buildAimAndShootPhase(profile, shotComplete, shotValidation, exitComplete));
                phases.add(buildExitPhase(exitComplete));
            }
            case SAFE_SCORE_EXIT -> {
                phases.add(buildAdvanceDepotNoCollectPhase(outpostToDepot, reachedDepot, exitComplete));
                phases.add(
                        buildTransitAndPreSpinPhase(profile, depotToShootingPose, reachedShootingPose, exitComplete));
                phases.add(buildAimAndShootPhase(profile, shotComplete, shotValidation, exitComplete));
                phases.add(buildExitPhase(exitComplete));
            }
            case CONSERVATIVE_MOBILITY -> phases.add(buildExitPhase(exitComplete));
            case ADAPTIVE_CLASSICAL -> {
                phases.add(
                        buildAdaptiveCollectOrBreakoffPhase(
                                profile,
                                outpostToDepot,
                                collectionValidation,
                                reachedDepot,
                                exitComplete,
                                autoStartTimestampSeconds));
                phases.add(
                        buildAdaptiveTransitOrBreakoffPhase(
                                profile,
                                depotToShootingPose,
                                reachedShootingPose,
                                exitComplete,
                                autoStartTimestampSeconds));
                phases.add(buildAimAndShootPhase(profile, shotComplete, shotValidation, exitComplete));
                phases.add(
                        buildAdaptiveFinishPhase(
                                autoStartTimestampSeconds.get(),
                                exitComplete));
            }
        }

        routine.active().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> autoStartTimestampSeconds.set(Timer.getFPGATimestamp())),
                        autoOrchestrator.runPhases(
                                profile.chooserName(),
                                plannerClient.linkState(),
                                routineMode,
                                phases,
                                forcedFallbackCondition,
                                "PLANNER_LINK_FALLBACK",
                                "Planner active mode lost healthy/fresh plan state; switching to local fallback.")));

        return routine;
    }

    private AutoPhase buildSeedAndDepartPhase(
            AutoTrajectory startToOutpost,
            AtomicReference<ValidationResult> traversalValidation,
            AtomicBoolean reachedOutpost,
            AtomicBoolean exitComplete) {
        final Command phaseCommand = Commands.sequence(
                Commands.runOnce(() -> traversalValidation.set(
                        safetyGate.validateAutoTraversalPose(autoContext.robotPose(), autoContext.alliance()))),
                Commands.either(
                        Commands.sequence(
                                startToOutpost.resetOdometry(),
                                startToOutpost.cmd()),
                        Commands.none(),
                        () -> traversalValidation.get().decision() == Decision.ALLOW));
        return new CommandAutoPhase(
                "P01_SEED_AND_DEPART",
                kSeedAndDepartTimeoutSeconds,
                phaseCommand,
                reachedOutpost::get,
                buildSafeExitCommand(exitComplete));
    }

    private AutoPhase buildCollectDepotFuelPhase(
            AutoTrajectory outpostToDepot,
            AtomicReference<ValidationResult> collectionValidation,
            AtomicBoolean reachedDepot,
            AtomicBoolean exitComplete) {
        final Command phaseCommand = Commands.sequence(
                Commands.runOnce(() -> collectionValidation.set(
                        safetyGate.validateHubCollectionRisk(autoContext.robotPose()))),
                Commands.either(
                        Commands.deadline(
                                outpostToDepot.cmd(),
                                intake.intakeCommand()),
                        Commands.none(),
                        () -> collectionValidation.get().decision() == Decision.ALLOW));
        return new CommandAutoPhase(
                "P02_COLLECT_DEPOT_FUEL",
                kCollectDepotTimeoutSeconds,
                phaseCommand,
                reachedDepot::get,
                buildSafeExitCommand(exitComplete));
    }

    private AutoPhase buildAdvanceDepotNoCollectPhase(
            AutoTrajectory outpostToDepot,
            AtomicBoolean reachedDepot,
            AtomicBoolean exitComplete) {
        final Command phaseCommand = Commands.sequence(
                Commands.runOnce(() -> intake.set(Intake.Position.STOWED)),
                outpostToDepot.cmd());
        return new CommandAutoPhase(
                "P02_ADVANCE_DEPOT_NO_COLLECT",
                kAdvanceDepotTimeoutSeconds,
                phaseCommand,
                reachedDepot::get,
                buildSafeExitCommand(exitComplete));
    }

    private AutoPhase buildTransitAndPreSpinPhase(
            LocalAutoProfile profile,
            AutoTrajectory depotToShootingPose,
            AtomicBoolean reachedShootingPose,
            AtomicBoolean exitComplete) {
        final Command phaseCommand = Commands.deadline(
                depotToShootingPose.cmd(),
                limelight.idle(),
                Commands.sequence(
                        Commands.waitSeconds(0.25),
                        Commands.parallel(
                                shooter.spinUpCommand(profile.shooterRpm()).withTimeout(2.0),
                                hood.positionCommand(profile.hoodPosition()))));
        return new CommandAutoPhase(
                "P03_TRANSIT_AND_PRESPIN",
                kTransitAndPreSpinTimeoutSeconds,
                phaseCommand,
                reachedShootingPose::get,
                buildSafeExitCommand(exitComplete));
    }

    private AutoPhase buildAimAndShootPhase(
            LocalAutoProfile profile,
            AtomicBoolean shotComplete,
            AtomicReference<ValidationResult> shotValidation,
            AtomicBoolean exitComplete) {
        return new CommandAutoPhase(
                "P04_AIM_AND_SHOOT",
                kAimAndShootTimeoutSeconds,
                buildShotPhaseCommand(profile, shotComplete, shotValidation),
                shotComplete::get,
                buildSafeExitCommand(exitComplete));
    }

    private AutoPhase buildExitPhase(AtomicBoolean exitComplete) {
        return new CommandAutoPhase(
                "P99_SAFE_EXIT",
                kSafeExitTimeoutSeconds,
                buildSafeExitCommand(exitComplete),
                exitComplete::get,
                buildSafeExitCommand(exitComplete));
    }

    private AutoPhase buildAdaptiveCollectOrBreakoffPhase(
            LocalAutoProfile profile,
            AutoTrajectory outpostToDepot,
            AtomicReference<ValidationResult> collectionValidation,
            AtomicBoolean reachedDepot,
            AtomicBoolean exitComplete,
            AtomicReference<Double> autoStartTimestampSeconds) {
        final Command collectTrajectoryCommand = Commands.sequence(
                Commands.deadline(
                        outpostToDepot.cmd(),
                        intake.intakeCommand()),
                Commands.runOnce(() -> reachedDepot.set(true)));
        final Command breakoffCommand = buildBreakoffDriveCommand(
                this::depotBreakoffPoseForAlliance,
                reachedDepot,
                2.2);
        final Command phaseCommand = Commands.sequence(
                Commands.runOnce(() -> collectionValidation.set(
                        safetyGate.validateHubCollectionRisk(autoContext.robotPose()))),
                Commands.either(
                        collectTrajectoryCommand,
                        breakoffCommand,
                        () -> shouldRunCollectionBranch(profile, collectionValidation.get(),
                                autoStartTimestampSeconds.get())));
        return new CommandAutoPhase(
                "P02_ADAPTIVE_COLLECT_OR_BREAKOFF",
                kCollectDepotTimeoutSeconds,
                phaseCommand,
                reachedDepot::get,
                buildSafeExitCommand(exitComplete));
    }

    private AutoPhase buildAdaptiveTransitOrBreakoffPhase(
            LocalAutoProfile profile,
            AutoTrajectory depotToShootingPose,
            AtomicBoolean reachedShootingPose,
            AtomicBoolean exitComplete,
            AtomicReference<Double> autoStartTimestampSeconds) {
        final Command trajectoryTransit = Commands.sequence(
                Commands.deadline(
                        depotToShootingPose.cmd(),
                        limelight.idle(),
                        Commands.sequence(
                                Commands.waitSeconds(0.25),
                                Commands.parallel(
                                        shooter.spinUpCommand(profile.shooterRpm()).withTimeout(2.0),
                                        hood.positionCommand(profile.hoodPosition())))),
                Commands.runOnce(() -> reachedShootingPose.set(true)));
        final Command breakoffTransit = Commands.sequence(
                Commands.deadline(
                        buildBreakoffDriveCommand(this::shootingBreakoffPoseForAlliance, reachedShootingPose, 2.6),
                        Commands.sequence(
                                Commands.waitSeconds(0.15),
                                Commands.parallel(
                                        shooter.spinUpCommand(profile.shooterRpm()).withTimeout(2.0),
                                        hood.positionCommand(profile.hoodPosition())))),
                Commands.runOnce(() -> reachedShootingPose.set(true)));
        final Command phaseCommand = Commands.either(
                trajectoryTransit,
                breakoffTransit,
                () -> shouldUseTrajectoryTransit(profile, autoStartTimestampSeconds.get()));
        return new CommandAutoPhase(
                "P03_ADAPTIVE_TRANSIT_OR_BREAKOFF",
                kTransitAndPreSpinTimeoutSeconds,
                phaseCommand,
                reachedShootingPose::get,
                buildSafeExitCommand(exitComplete));
    }

    private AutoPhase buildAdaptiveFinishPhase(
            Double autoStartTimestampSeconds,
            AtomicBoolean exitComplete) {
        final edu.wpi.first.wpilibj2.command.Command safeExitCommand = edu.wpi.first.wpilibj2.command.Commands.sequence(
                buildBreakoffDriveCommand(this::safeExitBreakoffPoseForAlliance, exitComplete, 1.4),
                buildSafeExitCommand(exitComplete));
        return new CommandAutoPhase(
                "P05_ADAPTIVE_FINISH",
                3.0,
                safeExitCommand,
                exitComplete::get,
                buildSafeExitCommand(exitComplete));
    }

    private Command buildShotPhaseCommand(
            LocalAutoProfile profile,
            AtomicBoolean shotComplete,
            AtomicReference<ValidationResult> shotValidation) {
        return Commands.sequence(
                Commands.runOnce(() -> shotValidation.set(
                        safetyGate.validateHubShot(autoContext.robotPose(), autoContext.alliance()))),
                Commands.either(
                        Commands.sequence(
                                Commands.parallel(
                                        new AimAndDriveCommand(swerve, () -> 0.0, () -> 0.0).withTimeout(1.4),
                                        shooter.spinUpCommand(profile.shooterRpm()),
                                        hood.positionCommand(profile.hoodPosition())),
                                Commands.parallel(
                                        feeder.feedCommand().withTimeout(0.8),
                                        floor.feedCommand().withTimeout(0.8),
                                        intake.agitateCommand().withTimeout(0.8)),
                                Commands.runOnce(() -> shotComplete.set(true))),
                        Commands.none(),
                        () -> shotValidation.get().decision() == Decision.ALLOW));
    }

    private Command buildBreakoffDriveCommand(
            java.util.function.Supplier<Pose2d> targetPoseSupplier,
            AtomicBoolean completionFlag,
            double timeoutSeconds) {
        return Commands.sequence(
                new DriveToPoseCommand(swerve, targetPoseSupplier).withTimeout(timeoutSeconds),
                Commands.runOnce(() -> completionFlag.set(true)));
    }

    private boolean shouldRunCollectionBranch(
            LocalAutoProfile profile,
            ValidationResult collectionValidation,
            Double autoStartTimestampSeconds) {
        if (profile.style() != LocalAutoStyle.ADAPTIVE_CLASSICAL) {
            return collectionValidation.decision() == Decision.ALLOW;
        }
        if (collectionValidation.decision() != Decision.ALLOW) {
            return false;
        }
        if (autoStartTimestampSeconds == null) {
            return false;
        }
        if (Timer.getFPGATimestamp() - autoStartTimestampSeconds > kAdaptiveCollectionMaxElapsedSeconds) {
            return false;
        }
        final Optional<DriverStation.Alliance> alliance = autoContext.alliance();
        if (alliance.isEmpty()) {
            return false;
        }
        final ShiftGameData shiftData = ShiftGameData.parse(DriverStation.getGameSpecificMessage());
        return isOwnShiftInactive(shiftData, alliance.get());
    }

    private boolean shouldUseTrajectoryTransit(LocalAutoProfile profile, Double autoStartTimestampSeconds) {
        if (profile.style() != LocalAutoStyle.ADAPTIVE_CLASSICAL) {
            return true;
        }
        if (autoStartTimestampSeconds == null) {
            return true;
        }
        final double elapsedSeconds = Timer.getFPGATimestamp() - autoStartTimestampSeconds;
        return elapsedSeconds <= kAdaptiveTrajectoryTransitMaxElapsedSeconds;
    }

    private Pose2d depotBreakoffPoseForAlliance() {
        return allianceAdjustedPose(OutpostAndDepotTrajectory$1.endPoseBlue());
    }

    private Pose2d shootingBreakoffPoseForAlliance() {
        return allianceAdjustedPose(OutpostAndDepotTrajectory$2.endPoseBlue());
    }

    private Pose2d safeExitBreakoffPoseForAlliance() {
        return allianceAdjustedPose(OutpostAndDepotTrajectory$3.endPoseBlue());
    }

    private Pose2d allianceAdjustedPose(Pose2d bluePose) {
        final Optional<DriverStation.Alliance> alliance = autoContext.alliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            final double mirroredX = kFieldLengthMeters - bluePose.getX();
            final Rotation2d mirroredHeading = Rotation2d.fromRadians(Math.PI - bluePose.getRotation().getRadians());
            return new Pose2d(mirroredX, bluePose.getY(), mirroredHeading);
        }
        return bluePose;
    }

    private PlannerExecutionMode effectivePlannerMode() {
        if (configuredPlannerMode == PlannerExecutionMode.LOCAL_ONLY) {
            return PlannerExecutionMode.LOCAL_ONLY;
        }
        if (!Constants.Autonomous.kUseBcnpPlanner) {
            return PlannerExecutionMode.LOCAL_ONLY;
        }
        if (configuredPlannerMode == PlannerExecutionMode.SHADOW) {
            return PlannerExecutionMode.SHADOW;
        }
        // ACTIVE mode only has authority when link is healthy.
        return plannerClient.isHealthy() ? PlannerExecutionMode.ACTIVE : PlannerExecutionMode.SHADOW;
    }

    private boolean shouldForceFallbackToLocal(LocalAutoProfile profile, PlannerExecutionMode routineMode) {
        if (routineMode != PlannerExecutionMode.ACTIVE) {
            return false;
        }
        if (!plannerClient.isHealthy()) {
            return true;
        }

        final Optional<AutoPlannerClient.RemotePlan> remotePlan = plannerClient.latestPlan();
        if (remotePlan.isEmpty()) {
            return true;
        }
        final AutoPlannerClient.RemotePlan plan = remotePlan.get();
        final long now = System.currentTimeMillis();
        final boolean policySourceValidForActive = plan.policySource() == AutoPlannerClient.POLICY_SOURCE_LOCAL
                || plan.policySource() == AutoPlannerClient.POLICY_SOURCE_CLASSICAL
                || plan.policySource() == AutoPlannerClient.POLICY_SOURCE_LEARNED_ACTIVE;
        return plan.profileId() != profile.profileId()
                || !plan.isFresh(now, Constants.Autonomous.kBcnpPlanFreshMs)
                || plan.phaseCount() <= 0
                || plan.globalConfidencePermille() < Constants.Autonomous.kActivePlanMinConfidencePermille
                || !policySourceValidForActive;
    }

    private ShadowAgreement computeShadowAgreement(Optional<AutoPlannerClient.RemotePlan> remotePlan) {
        if (remotePlan.isEmpty()) {
            return new ShadowAgreement(false, "No remote plan observed yet.");
        }
        final AutoPlannerClient.RemotePlan plan = remotePlan.get();
        if (!plan.isFresh(System.currentTimeMillis(), Constants.Autonomous.kBcnpPlanFreshMs)) {
            return new ShadowAgreement(false, "Latest remote plan is stale.");
        }
        final int localPhaseCount = expectedPhaseCount(plan.profileId());
        if (localPhaseCount <= 0) {
            return new ShadowAgreement(false, "Profile is unknown to local routine map.");
        }
        if (plan.phaseCount() != localPhaseCount) {
            return new ShadowAgreement(
                    false,
                    "Phase-count mismatch local=" + localPhaseCount + " remote=" + plan.phaseCount());
        }
        final int expectedObjective = expectedObjectiveId(plan.profileId());
        if (expectedObjective > 0 && plan.objectiveId() != 0 && plan.objectiveId() != expectedObjective) {
            return new ShadowAgreement(
                    false,
                    "Objective mismatch local=" + expectedObjective + " remote=" + plan.objectiveId());
        }
        return new ShadowAgreement(true, "Remote and local phase budgets match.");
    }

    private int expectedPhaseCount(int profileId) {
        final LocalAutoProfile profile = profileForId(profileId);
        if (profile != null) {
            return switch (profile.style()) {
                case MAX_FUEL -> 5;
                case FAST_CYCLE_SCORE -> 5;
                case SAFE_SCORE_EXIT -> 5;
                case CONSERVATIVE_MOBILITY -> 2;
                case ADAPTIVE_CLASSICAL -> 5;
            };
        }
        return -1;
    }

    private int expectedObjectiveId(int profileId) {
        final LocalAutoProfile profile = profileForId(profileId);
        return profile == null ? -1 : profile.objectiveId();
    }

    private Command buildSafeExitCommand(AtomicBoolean exitComplete) {
        return Commands.sequence(
                Commands.runOnce(shooter::stop),
                Commands.runOnce(() -> feeder.setPercentOutput(0)),
                Commands.runOnce(() -> floor.set(Floor.Speed.STOP)),
                Commands.runOnce(() -> intake.set(Intake.Speed.STOP)),
                Commands.runOnce(() -> intake.set(Intake.Position.STOWED)),
                Commands.runOnce(() -> exitComplete.set(true)));
    }

    private AutoRoutine outpostAndDepotLegacyRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");
        final AutoTrajectory startToOutpost = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);

        routine.active().onTrue(
                // First leg always seeds odometry from the authored trajectory start pose.
                Commands.sequence(
                        startToOutpost.resetOdometry(),
                        startToOutpost.cmd()));

        routine.observe(hanger::isHomed).onTrue(
                // Placeholder for any "post-home" autonomous prep actions.
                Commands.sequence(
                        Commands.waitSeconds(0.5)));

        startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd());

        // Begin intake while approaching depot so pieces are acquired during transit.
        outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd());

        // Keep Limelight subsystem scheduled so network table updates are fresh during
        // this leg.
        depotToShootingPose.active().whileTrue(limelight.idle());
        depotToShootingPose.atTime(0.5).onTrue(
                // Pre-spin shooter and pre-position hood before final aim-and-fire sequence.
                Commands.parallel(
                        shooter.spinUpCommand(2600),
                        hood.positionCommand(0.32)));
        depotToShootingPose.done().onTrue(
                subsystemCommands.aimAndShoot()
                        .withTimeout(5));

        return routine;
    }

    private record LocalAutoProfile(
            String chooserName,
            int profileId,
            double shooterRpm,
            double hoodPosition,
            int objectiveId,
            LocalAutoStyle style) {
    }

    private enum LocalAutoStyle {
        MAX_FUEL,
        FAST_CYCLE_SCORE,
        SAFE_SCORE_EXIT,
        CONSERVATIVE_MOBILITY,
        ADAPTIVE_CLASSICAL
    }

    private record ShadowAgreement(boolean agrees, String reason) {
    }
}
