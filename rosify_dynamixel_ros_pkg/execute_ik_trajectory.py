#!/usr/bin/env python3
"""Two-stage robot motion: jtraj to 50 mm above target, then straight-down ctraj.

DRY RUN by default. XYZ in metres in the robot's DH-base frame.
The tool's +Z axis must remain close to world -Z, including during jtraj.
This is NOT a collision checker or a hardware emergency stop.
"""
import argparse
import time
import numpy as np
import dynamixel_sdk as dxl
from spatialmath import SE3
from roboticstoolbox import jtraj, ctraj
from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import (
    MOTOR_IDS, MOTOR_LIMITS, motor_to_joint, joint_to_motor
)

PORT = '/dev/ttyUSB0'
BAUD = 1000000
ADDR_MODE, ADDR_TORQUE, ADDR_HW = 11, 64, 70
ADDR_ACCEL, ADDR_VEL, ADDR_GOAL, ADDR_PRESENT = 108, 112, 116, 132
FREQ_HZ = 5
STAGE1_S = 5.0
STAGE2_S = 5.0
PROFILE_ACCEL = 500
PROFILE_VEL = 3000  # cap; actual travel may take longer under load
MAX_TILT_DEG = 10
MAX_POSITION_ERROR_MM = 2.0
MAX_FOLLOWING_STEPS = 30
MAX_STEP_JUMP = 60  # per sampled command, incl. first
MIN_MODEL_TCP_Z = -0.075  # DH frame only, NOT an actual table clearance


def check_limits(positions):
    for mid, pos in zip(MOTOR_IDS, positions):
        lo, hi = MOTOR_LIMITS[mid]
        if not lo <= int(pos) <= hi:
            raise RuntimeError(f'Motor {mid}: {pos} outside [{lo}, {hi}]')


def validate_pose(robot, q, desired_xyz=None):
    T = robot.fkine(q)
    z_axis = T.R[:, 2]
    tilt = np.rad2deg(np.arccos(np.clip(np.dot(z_axis, [0, 0, -1]), -1, 1)))
    if tilt > MAX_TILT_DEG:
        raise RuntimeError(f'Tool tilt {tilt:.2f} deg exceeds {MAX_TILT_DEG} deg')
    if T.t[2] < MIN_MODEL_TCP_Z:
        raise RuntimeError(f'TCP model Z={T.t[2]:.3f} m below guard (DH frame)')
    if desired_xyz is not None:
        err = np.linalg.norm(T.t - desired_xyz) * 1000
        if err > MAX_POSITION_ERROR_MM:
            raise RuntimeError(f'Cartesian error {err:.2f} mm at path point')


def plan(robot, start_motors, target_xyz):
    q_start = motor_to_joint(start_motors)

# Bruk faktiske motorgrenser som IK-leddgrenser
    for i, mid in enumerate(MOTOR_IDS):

        motor_min, motor_max = MOTOR_LIMITS[mid]

        p_min = start_motors.copy()
        p_max = start_motors.copy()

        p_min[i] = motor_min
        p_max[i] = motor_max

        q_min = motor_to_joint(p_min)[i]
        q_max = motor_to_joint(p_max)[i]

        robot.links[i].qlim = [
            min(q_min, q_max),
            max(q_min, q_max)
        ]

        above = target_xyz + np.array([0., 0., 0.0])
        R_down = SE3.Rx(np.pi)
        mask = [1, 1, 1, 1, 1, 0]

    def solve(xyz, seed):
        result = robot.ikine_LM(SE3.Trans(*xyz) * R_down,
                                q0=seed, mask=mask, joint_limits=True)
        if not result.success:
            print("\n========== IK FAILED ==========")
            print("XYZ [mm]:", np.round(xyz * 1000, 2))
            print("Last valid joint angles [deg]:",
                np.round(np.rad2deg(seed), 2))

            print("Last valid motor positions:")

            for mid, p in zip(MOTOR_IDS, joint_to_motor(seed)):
                lo, hi = MOTOR_LIMITS[mid]

                print(
                    f"Motor {mid}: {p} "
                    f"| min={lo} | max={hi}"
                )

            print("IK reason:", result.reason)

            raise RuntimeError(
                f'IK not solved for {np.round(xyz, 3)}'
            )
        q = np.asarray(result.q, dtype=float).copy()
        q[4] = q_start[4]  # Motor 5 does not receive commands
        validate_pose(robot, q, xyz)
        return q

    # Phase 1: joint-space trajectory, may tilt tool. Reject if it does.
    q_above = solve(above, q_start)
    n1 = int(STAGE1_S * FREQ_HZ)
    first = jtraj(q_start, q_above, np.linspace(0, STAGE1_S, n1 + 1)).q

    # Phase 2: Cartesian line straight down. IK at every sample.
    n2 = int(STAGE2_S * FREQ_HZ)
    poses = ctraj(SE3.Trans(*above)*R_down,
                  SE3.Trans(*target_xyz)*R_down,
                  s=np.linspace(0, 1, n2 + 1))
    second = []
    q_prev = q_above
    for T in poses:
        q_prev = solve(np.asarray(T.t).reshape(3), q_prev)
        second.append(q_prev.copy())

    # Preserve the actual initial motor command at sample zero.
    qs = np.vstack([first, np.asarray(second)[1:]])
    paths = np.asarray([joint_to_motor(q) for q in qs], dtype=int)
    paths[:, 4] = start_motors[4]
    for index, (q, p) in enumerate(zip(qs, paths)):
        check_limits(p)
        validate_pose(robot, motor_to_joint(p))
        if index and np.max(np.abs(paths[index, :4] - paths[index-1, :4])) > MAX_STEP_JUMP:
            raise RuntimeError(f'Motor step jump too large at sample {index}')
    # Verify the rounded motor targets along the vertical phase really go down.
    for i in range(n1, len(paths)):
        target_sample = above + (target_xyz - above) * ((i-n1) / n2)
        q_actual = motor_to_joint(paths[i])
        err = np.linalg.norm(robot.fkine(q_actual).t - target_sample) * 1000
        if err > MAX_POSITION_ERROR_MM:
            raise RuntimeError(f'Rounded motor position at sample {i}: {err:.2f} mm from line')
    return paths, qs, above


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True, help='Target X, DH-base metres')
    parser.add_argument('--y', type=float, required=True, help='Target Y, DH-base metres')
    parser.add_argument('--z', type=float, required=True, help='Target Z, DH-base metres')
    parser.add_argument('--execute', action='store_true', help='Enable manual-confirmed movement')
    args = parser.parse_args()
    target = np.array([args.x, args.y, args.z])
    # Kalibrering av fysisk TCP-avvik
    XY_OFFSET = np.array([
        -0.00,  # X-korreksjon i meter
       -0.005,  # Y-korreksjon i meter
        0.000
    ])

    target = target + XY_OFFSET
    if not np.all(np.isfinite(target)):
        parser.error('Coordinates must be finite')

    port = dxl.PortHandler(PORT)
    packet = dxl.PacketHandler(2.0)

    def checked(result, error, name):
        if result != dxl.COMM_SUCCESS or error:
            raise RuntimeError(f'{name}: {packet.getTxRxResult(result)}; {packet.getRxPacketError(error)}')

    def read1(mid, address):
        value, result, error = packet.read1ByteTxRx(port, mid, address)
        checked(result, error, f'Read ID={mid}, addr={address}')
        return value

    def read4(mid, address):
        value, result, error = packet.read4ByteTxRx(port, mid, address)
        checked(result, error, f'Read ID={mid}, addr={address}')
        return value if value < 2**31 else value-2**32

    def write4(mid, address, value):
        result, error = packet.write4ByteTxRx(port, mid, address, int(value))
        checked(result, error, f'Write ID={mid}, addr={address}')

    def get_positions():
        return np.array([read4(mid, ADDR_PRESENT) for mid in MOTOR_IDS], dtype=int)

    if not port.openPort():
        raise RuntimeError(f'Could not open {PORT}')
    try:
        if not port.setBaudRate(BAUD):
            raise RuntimeError('Could not set baud rate')
        start = get_positions()
        check_limits(start)
        robot = create_robot()
        robot.tool = SE3.Tz(0.080)
        path, _, above = plan(robot, start, target)
        print('\n=== TWO-STAGE IK PLAN ===')
        print('Start motors:', start.tolist())
        print('TCP start [mm]:', np.round(robot.fkine(motor_to_joint(start)).t*1000, 2))
        print('50 mm above [mm]:', np.round(above*1000, 2))
        print('Target [mm]:', np.round(target*1000, 2))
        print('Goal motors:', path[-1].tolist())
        print('Motor delta:', (path[-1]-start).tolist())
        print('Samples:', len(path), '| Nominal time:', STAGE1_S+STAGE2_S, 's')
        print('Allowed tool tilt throughout path:', MAX_TILT_DEG, 'degrees')
        print('No obstacle/cable/link collision checking; no real-world plate clearance check')
        print('DRY RUN' if not args.execute else 'PHYSICAL EXECUTION REQUESTED')
        if not args.execute:
            return

        for mid in MOTOR_IDS:
            if read1(mid, ADDR_MODE) != (5 if mid == 5 else 3):
                raise RuntimeError(f'ID {mid}: wrong operating mode')
            if read1(mid, ADDR_TORQUE) != 1:
                raise RuntimeError(f'ID {mid}: torque is off')
            if read1(mid, ADDR_HW) != 0:
                raise RuntimeError(f'ID {mid}: hardware error')
        print('WARNING: Ctrl+C does NOT stop already commanded motors. Have hardware E-stop ready.')
        print('Check entire arm, pen and cable clearances for BOTH phases.')
        if input('Type MOVE ONCE to confirm: ') != 'MOVE ONCE':
            print('Cancelled')
            return
        fresh = get_positions()
        if np.max(np.abs(fresh-start)) > 2:
            raise RuntimeError('Robot moved since planning; re-run')
        print("\nDynamixel motion profiles:")

        for mid in MOTOR_IDS[:4]:
            drive_mode = read1(mid, 10)
            profile_velocity = read4(mid, 112)
            profile_acceleration = read4(mid, 108)

            print(
                f"Motor {mid}: "
                f"Drive Mode={drive_mode}, "
                f"Time-based={bool(drive_mode & 0x04)}, "
                f"Velocity={profile_velocity}, "
                f"Acceleration={profile_acceleration}"
            )

        for mid in MOTOR_IDS[:4]:
            write4(mid, ADDR_ACCEL, PROFILE_ACCEL)
            write4(mid, ADDR_VEL, PROFILE_VEL)

        
        sync = dxl.GroupSyncWrite(port, packet, ADDR_GOAL, 4)

        def send_goal(positions):
            sync.clearParam()

            for mid, goal in zip(MOTOR_IDS[:4], positions[:4]):
                v = int(goal)
                payload = [
                    (v >> shift) & 0xff
                    for shift in (0, 8, 16, 24)
                ]

                if not sync.addParam(mid, payload):
                    raise RuntimeError(
                        f"SyncWrite failed for motor {mid}"
                    )

            result = sync.txPacket()
            sync.clearParam()

            if result != dxl.COMM_SUCCESS:
                raise RuntimeError(
                    packet.getTxRxResult(result)
                )

        # =====================================
        # FASE 1: MJUK RØRSLE TIL PUNKTET OVER
        # =====================================

        n1 = int(STAGE1_S * FREQ_HZ)
        above_motors = path[n1]

        
        print("\nFASE 1: Moving 5 cm above target")

        # Start klokka éin gong, før målkommandoen
        phase1_start = time.monotonic()

        send_goal(above_motors)

        arrived = False
        tolerance = np.array([3, 12, 15, 3])

        # Maks 12 sekund frå målkommandoen
        while time.monotonic() - phase1_start < 12.0:

            time.sleep(0.2)

            measured = get_positions()
            error = measured[:4] - above_motors[:4]

            elapsed = time.monotonic() - phase1_start

            within_tolerance = np.all(
                np.abs(error) <= tolerance
            )

            print(
                f"Phase 1: {elapsed:.1f}s | "
                f"error={error.tolist()} | "
                f"within_tolerance={within_tolerance}"
            )

            if any(read1(mid, ADDR_HW) for mid in MOTOR_IDS):
                raise RuntimeError("Motor hardware error")

            # Vent til minst 5 sekund har gått,
            # OG motorane er innanfor toleransen
            if elapsed >= 4.0 and within_tolerance:
                arrived = True
                break

        if not arrived:
            raise RuntimeError(
                "Phase 1 target not reached. "
                "Phase 2 cancelled."
            )

        # Kontroller faktisk TCP og orientering
        measured = get_positions()
        actual_q = motor_to_joint(measured)

        validate_pose(robot, actual_q)

        actual_xyz = np.asarray(
            robot.fkine(actual_q).t
        )

        print(
            "Actual TCP [mm]:",
            np.round(actual_xyz * 1000, 2)
        )

        print(
            "Expected TCP [mm]:",
            np.round(above * 1000, 2)
        )

        tcp_error = np.linalg.norm(actual_xyz - above)

        print(
            "TCP error [mm]:",
            round(tcp_error * 1000, 2)
        )
        tcp_difference = (actual_xyz - above) * 1000

        print("TCP deviation XYZ [mm]:",
            np.round(tcp_difference, 2))

        print("TCP deviation total [mm]:",
            round(tcp_error * 1000, 2))

        # Finn den modellerte TCP-posisjonen basert på faktisk motorstilling
        actual_q = motor_to_joint(measured)
        actual_xyz = np.asarray(robot.fkine(actual_q).t).reshape(3)

        # Undersøk ein korrigert vertikal bane frå den faktiske posisjonen
        corrected_target = np.array([
            actual_xyz[0],
            actual_xyz[1],
            target[2]
        ])

        print("\n--- POSSIBLE PHASE 2 REPLAN ---")
        print("Actual start XYZ [mm]:",
            np.round(actual_xyz * 1000, 2))
        print("Original target XYZ [mm]:",
            np.round(target * 1000, 2))
        print("Vertical target from actual XY [mm]:",
            np.round(corrected_target * 1000, 2))
        print("Required Z movement [mm]:",
            round((corrected_target[2] - actual_xyz[2]) * 1000, 2))
        print("No new motor commands sent.")

        
        # ========================================
        # PLANLEGG FASE 2 FRÅ FAKTISK MOTORSTILLING
        # ========================================

        print("\nReplanning phase 2 from actual position")

        # Les motorane på nytt
        measured = get_positions()
        actual_q = motor_to_joint(measured)

        # Hald pennen vertikalt nedover
        R_down = SE3.Rx(np.pi)

        # Originalt XYZ-mål
        T_goal = SE3.Trans(*target) * R_down

        # Finn ny IK frå faktisk leddstilling
        sol = robot.ikine_LM(
            T_goal,
            q0=actual_q,
            mask=[1, 1, 1, 0.5, 0.5, 0.5],
            joint_limits=True
        )

        if not sol.success or sol.q is None:
            raise RuntimeError(
                "Phase 2 IK failed. No movement sent."
            )

        q_goal = np.asarray(sol.q, dtype=float).copy()
        q_goal[4] = actual_q[4]

        # Ny målposisjon for motorane
        final_goal = np.asarray(
            joint_to_motor(q_goal),
            dtype=int
        )
        final_goal[4] = measured[4]

        check_limits(final_goal)

        # Kontroller faktisk berekna sluttpose
        validate_pose(robot, motor_to_joint(final_goal), target)

        # Kontroller mellomstillingar langs leddrørsla
        # (ikkje full kollisjonskontroll)
        for alpha in np.linspace(0, 1, 51):
            p = np.rint(
                measured + alpha * (final_goal - measured)
            ).astype(int)

            check_limits(p)
            validate_pose(robot, motor_to_joint(p))

            z = robot.fkine(motor_to_joint(p)).t[2]

            if z < target[2] - 0.002:
                raise RuntimeError(
                    "Phase 2 would dip below target Z."
                )

        print("Current motors:", measured.tolist())
        print("New motor goal:", final_goal.tolist())
        print("Motor changes:", (final_goal - measured).tolist())
        print("Current TCP [mm]:",
              np.round(robot.fkine(actual_q).t * 1000, 2))
        print("Target TCP [mm]:",
              np.round(target * 1000, 2))

        print("\nPhase 2 IK recalculated successfully")
        print("No phase 2 movement sent yet.")


        
        # =====================================
        # FASE 2: MJUK RØRSLE TIL SLUTTPUNKTET
        # =====================================

        print("\nFASE 2: Moving to final target")

        # Bruk siste planlagde motorposisjon
        # final_goal = path[-1]

        # 5 sekund med mjuk akselerasjon
        PHASE2_TIME = 5000
        PHASE2_ACCEL = 1000

        # Konfigurer tidsbasert profil
        for mid in MOTOR_IDS[:4]:

            drive_mode = read1(mid, 10)

            if not (drive_mode & 0x04):
                raise RuntimeError(
                    f"Motor {mid} must use time-based profile"
                )

            write4(mid, ADDR_ACCEL, PHASE2_ACCEL)
            write4(mid, ADDR_VEL, PHASE2_TIME)

        print("Moving slowly for approximately 5 seconds")

        # Send sluttmålet berre éin gong
        if input("Type PHASE 2 to execute replanned descent: ") != "PHASE 2":
            print("Phase 2 cancelled.")
            return

        # Kontroller at armen ikkje har flytta seg
        if np.max(np.abs(get_positions() - measured)) > 2:
            raise RuntimeError("Robot moved after replanning.")

        send_goal(final_goal)
        send_goal(final_goal)

        # Vent på rørsla
        time.sleep(5.5)

        # Les faktisk motorposisjon
        measured = get_positions()

        error = measured[:4] - final_goal[:4]

        print("Final motor positions:", measured.tolist())
        print("Final motor error:", error.tolist())

        # Kontroller maskinvarefeil
        for mid in MOTOR_IDS:
            if read1(mid, ADDR_HW) != 0:
                raise RuntimeError(
                    f"Hardware error on motor {mid}"
                )

        # Rekn ut faktisk modellert TCP
        actual_q = motor_to_joint(measured)
        actual_tcp = robot.fkine(actual_q).t

        print("Actual TCP [mm]:",
              np.round(actual_tcp * 1000, 2))

        print("Target TCP [mm]:",
              np.round(target * 1000, 2))

        print("FASE 2 complete")



        time.sleep(1.0)
        actual = get_positions()
        actual_tcp = robot.fkine(motor_to_joint(actual)).t
        print('Final motors:', actual.tolist())
        print('Goal errors [steps]:', (actual - path[-1]).tolist())
        print('Estimated TCP [mm]:', np.round(actual_tcp*1000, 2))
        print('Estimated goal error [mm]:', np.round((target-actual_tcp)*1000, 2))
        print('Done: no further commands. FK is model-based, not external measurement.')
    finally:
        port.closePort()


if __name__ == '__main__':
    main()
