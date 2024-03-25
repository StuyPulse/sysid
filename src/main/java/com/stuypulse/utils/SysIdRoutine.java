package com.stuypulse.utils;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static java.util.Map.entry;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Map;
import java.util.function.Consumer;

/**
 * A SysId characterization routine for a single mechanism. Mechanisms may have multiple motors.
 *
 * <p>A single subsystem may have multiple mechanisms, but mechanisms should not share test
 * routines. Each complete test of a mechanism should have its own SysIdRoutine instance, since the
 * log name of the recorded data is determined by the mechanism name.
 *
 * <p>The test state (e.g. "quasistatic-forward") is logged once per iteration during test
 * execution, and once with state "none" when a test ends. Motor frames are logged every iteration
 * during test execution.
 *
 * <p>Timestamps are not coordinated across data, so motor frames and test state tags may be
 * recorded on different log frames. Because frame alignment is not guaranteed, SysId parses the log
 * by using the test state flag to determine the timestamp range for each section of the test, and
 * then extracts the motor frames within the valid timestamp ranges. If a given test was run
 * multiple times in a single logfile, the user will need to select which of the tests to use for
 * the fit in the analysis tool.
 */
public class SysIdRoutine extends SysIdRoutineLog {
  private final Config m_config;
  private final Mechanism m_mechanism;
  private final MutableMeasure<Current> m_outputVolts = mutable(Amps.of(0));
  private final Consumer<State> m_recordState;

  /**
   * Create a new SysId characterization routine.
   *
   * @param config Hardware-independent parameters for the SysId routine.
   * @param mechanism Hardware interface for the SysId routine.
   */
  public SysIdRoutine(Config config, Mechanism mechanism) {
    super(mechanism.m_name);
    m_config = config;
    m_mechanism = mechanism;
    m_recordState = config.m_recordState != null ? config.m_recordState : this::recordState;
  }

  /** Hardware-independent configuration for a SysId test routine. */
  public static class Config {
    /** The amperage ramp rate used for quasistatic test routines. */
    public final Measure<Velocity<Current>> m_rampRate;

    /** The step amperage output used for dynamic test routines. */
    public final Measure<Current> m_stepCurrent;

    /** Safety timeout for the test routine commands. */
    public final Measure<Time> m_timeout;

    /** Optional handle for recording test state in a third-party logging solution. */
    public final Consumer<State> m_recordState;

    /**
     * Create a new configuration for a SysId test routine.
     *
     * @param rampRate The amperage ramp rate used for quasistatic test routines. Defaults to 2 amps
     *     per second if left null.
     * @param stepAmperage The step amperage output used for dynamic test routines. Defaults to 60
     *     amps if left null.
     * @param timeout Safety timeout for the test routine commands. Defaults to 10 seconds if left
     *     null.
     * @param recordState Optional handle for recording test state in a third-party logging
     *     solution. If provided, the test routine state will be passed to this callback instead of
     *     logged in WPILog.
     */
    public Config(
        Measure<Velocity<Current>> rampRate,
        Measure<Current> stepAmperage,
        Measure<Time> timeout,
        Consumer<State> recordState) {
      m_rampRate = rampRate != null ? rampRate : Amps.of(2).per(Seconds.of(1));
      m_stepCurrent = stepAmperage != null ? stepAmperage : Amps.of(60);
      m_timeout = timeout != null ? timeout : Seconds.of(30);
      m_recordState = recordState;
    }

    /**
     * Create a new configuration for a SysId test routine.
     *
     * @param rampRate The amperage ramp rate used for quasistatic test routines. Defaults to 2 amps
     *     per second if left null.
     * @param stepAmperage The step amperage output used for dynamic test routines. Defaults to 60
     *     amps if left null.
     * @param timeout Safety timeout for the test routine commands. Defaults to 30 seconds if left
     *     null.
     */
    public Config(
        Measure<Velocity<Current>> rampRate, Measure<Current> stepAmperage, Measure<Time> timeout) {
      this(rampRate, stepAmperage, timeout, null);
    }

    /**
     * Create a default configuration for a SysId test routine with all default settings.
     *
     * <p>rampRate: 2 amps/sec
     *
     * <p>stepAmperage: 60 amps
     *
     * <p>timeout: 30 seconds
     */
    public Config() {
      this(null, null, null, null);
    }
  }

  /**
   * A mechanism to be characterized by a SysId routine. Defines callbacks needed for the SysId test
   * routine to control and record data from the mechanism.
   */
  public static class Mechanism {
    /** Sends the SysId-specified drive signal to the mechanism motors during test routines. */
    public final Consumer<Measure<Current>> m_drive;

    /**
     * Returns measured data (amperages, positions, velocities) of the mechanism motors during test
     * routines.
     */
    public final Consumer<SysIdRoutineLog> m_log;

    /** The subsystem containing the motor(s) that is (or are) being characterized. */
    public final Subsystem m_subsystem;

    /** The name of the mechanism being tested. */
    public final String m_name;

    /**
     * Create a new mechanism specification for a SysId routine.
     *
     * @param drive Sends the SysId-specified drive signal to the mechanism motors during test
     *     routines.
     * @param log Returns measured data of the mechanism motors during test routines. To return
     *     data, call `motor(string motorName)` on the supplied `SysIdRoutineLog` instance, and then
     *     call one or more of the chainable logging handles (e.g. `amperage`) on the returned
     *     `MotorLog`. Multiple motors can be logged in a single callback by calling `motor`
     *     multiple times.
     * @param subsystem The subsystem containing the motor(s) that is (or are) being characterized.
     *     Will be declared as a requirement for the returned test commands.
     * @param name The name of the mechanism being tested. Will be appended to the log entry title
     *     for the routine's test state, e.g. "sysid-test-state-mechanism". Defaults to the name of
     *     the subsystem if left null.
     */
    public Mechanism(
        Consumer<Measure<Current>> drive,
        Consumer<SysIdRoutineLog> log,
        Subsystem subsystem,
        String name) {
      m_drive = drive;
      m_log = log != null ? log : l -> {};
      m_subsystem = subsystem;
      m_name = name != null ? name : subsystem.getName();
    }

    /**
     * Create a new mechanism specification for a SysId routine. Defaults the mechanism name to the
     * subsystem name.
     *
     * @param drive Sends the SysId-specified drive signal to the mechanism motors during test
     *     routines.
     * @param log Returns measured data of the mechanism motors during test routines. To return
     *     data, call `motor(string motorName)` on the supplied `SysIdRoutineLog` instance, and then
     *     call one or more of the chainable logging handles (e.g. `amperage`) on the returned
     *     `MotorLog`. Multiple motors can be logged in a single callback by calling `motor`
     *     multiple times.
     * @param subsystem The subsystem containing the motor(s) that is (or are) being characterized.
     *     Will be declared as a requirement for the returned test commands. The subsystem's `name`
     *     will be appended to the log entry title for the routine's test state, e.g.
     *     "sysid-test-state-subsystem".
     */
    public Mechanism(
        Consumer<Measure<Current>> drive, Consumer<SysIdRoutineLog> log, Subsystem subsystem) {
      this(drive, log, subsystem, null);
    }
  }

  /** Motor direction for a SysId test. */
  public enum Direction {
    /** Forward. */
    kForward,
    /** Reverse. */
    kReverse
  }

  /**
   * Returns a command to run a quasistatic test in the specified direction.
   *
   * <p>The command will call the `drive` and `log` callbacks supplied at routine construction once
   * per iteration. Upon command end or interruption, the `drive` callback is called with a value of
   * 0 volts.
   *
   * @param direction The direction in which to run the test.
   * @return A command to run the test.
   */
  public Command quasistatic(Direction direction) {
    State state;
    if (direction == Direction.kForward) {
      state = State.kQuasistaticForward;
    } else { // if (direction == Direction.kReverse) {
      state = State.kQuasistaticReverse;
    }

    double outputSign = direction == Direction.kForward ? 1.0 : -1.0;

    Timer timer = new Timer();
    return m_mechanism
        .m_subsystem
        .runOnce(timer::restart)
        .andThen(
            m_mechanism.m_subsystem.run(
                () -> {
                  m_mechanism.m_drive.accept(
                      m_outputVolts.mut_replace(
                          outputSign * timer.get() * m_config.m_rampRate.in(Amps.per(Second)),
                          Amps));
                  m_mechanism.m_log.accept(this);
                  m_recordState.accept(state);
                }))
        .finallyDo(
            () -> {
              m_mechanism.m_drive.accept(Amps.of(0));
              m_recordState.accept(State.kNone);
              timer.stop();
            })
        .withName("sysid-" + state.toString() + "-" + m_mechanism.m_name)
        .withTimeout(m_config.m_timeout.in(Seconds));
  }

  /**
   * Returns a command to run a dynamic test in the specified direction.
   *
   * <p>The command will call the `drive` and `log` callbacks supplied at routine construction once
   * per iteration. Upon command end or interruption, the `drive` callback is called with a value of
   * 0 volts.
   *
   * @param direction The direction in which to run the test.
   * @return A command to run the test.
   */
  public Command dynamic(Direction direction) {
    double outputSign = direction == Direction.kForward ? 1.0 : -1.0;
    State state =
        Map.ofEntries(
                entry(Direction.kForward, State.kDynamicForward),
                entry(Direction.kReverse, State.kDynamicReverse))
            .get(direction);

    return m_mechanism
        .m_subsystem
        .runOnce(
            () -> m_outputVolts.mut_replace(m_config.m_stepCurrent.in(Amps) * outputSign, Amps))
        .andThen(
            m_mechanism.m_subsystem.run(
                () -> {
                  m_mechanism.m_drive.accept(m_outputVolts);
                  m_mechanism.m_log.accept(this);
                  m_recordState.accept(state);
                }))
        .finallyDo(
            () -> {
              m_mechanism.m_drive.accept(Amps.of(0));
              m_recordState.accept(State.kNone);
            })
        .withName("sysid-" + state.toString() + "-" + m_mechanism.m_name)
        .withTimeout(m_config.m_timeout.in(Seconds));
  }
}
