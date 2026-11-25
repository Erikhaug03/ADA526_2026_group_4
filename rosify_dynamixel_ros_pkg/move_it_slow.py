#!/usr/bin/env python3
"""SLOW, BOUNDED, one-shot Cartesian IK move for a five-axis Dynamixel arm.

DRY-RUN by default. Coordinates are in the robot DH BASE frame (metres),
NOT in physical-table coordinates. Not a collision checker or emergency stop.
Motor 3 has previously shown large following errors; this script never
retries or compensates automatically.
"""
import argparse
import time
import numpy as np
import dynamixel_sdk as dxl
from spatialmath import SE3
from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import (
    MOTOR_IDS, MOTOR_LIMITS, motor_to_joint, joint_to_motor
)

PORT = '/dev/ttyUSB1'
BAUD = 1000000
PROTOCOL = 2.0
TCP_OFFSET = 0.080
MAX_CARTESIAN_MOVE_M = 1   # At most 50 mm from CURRENT FK TCP position
MAX_MOTOR_STEPS = 1000           # At most 65 encoder counts on any moving axis
MIN_TCP_Z_DH_M = 0.050        # Only a model-frame sanity check, NOT clearance
MAX_IK_RESIDUAL_M = 0.002
PROFILE_VELOCITY = 3          # Keep previously tested slow motor profile
PROFILE_ACCELERATION = 2
ADDR_MODE, ADDR_TORQUE, ADDR_HW_ERROR = 11, 64, 70
ADDR_ACCEL, ADDR_VELOCITY, ADDR_GOAL, ADDR_PRESENT = 108, 112, 116, 132


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_argument_group('Target (choose absolute OR relative)')
    for name in ('x', 'y', 'z'):
        target.add_argument(f'--{name}', type=float, help='Absolute XYZ in DH-base METRES')
    for name in ('dx-mm', 'dy-mm', 'dz-mm'):
        target.add_argument(f'--{name}', type=float, help='Relative displacement in millimetres')
    parser.add_argument('--execute', action='store_true', help='Allow ONE manual-confirmed physical move')
    args = parser.parse_args()
    absolute = [args.x, args.y, args.z]
    relative = [args.dx_mm, args.dy_mm, args.dz_mm]
    if all(v is not None for v in absolute) and all(v is None for v in relative):
        mode = 'absolute'
    elif all(v is None for v in absolute) and all(v is not None for v in relative):
        mode = 'relative'
    else:
        parser.error('Specify either --x --y --z (metres) OR --dx-mm --dy-mm --dz-mm (millimetres)')
    requested = np.array(absolute if mode == 'absolute' else relative, dtype=float)
    if not np.all(np.isfinite(requested)):
        parser.error('All coordinates must be finite')

    port = dxl.PortHandler(PORT)
    packet = dxl.PacketHandler(PROTOCOL)

    def check(result, error, context):
        if result != dxl.COMM_SUCCESS or error:
            raise RuntimeError(f'{context}: {packet.getTxRxResult(result)}; '
                               f'{packet.getRxPacketError(error)}')

    def read1(mid, address):
        v, r, e = packet.read1ByteTxRx(port, mid, address)
        check(r, e, f'Read ID={mid}, address={address}')
        return int(v)

    def read4(mid, address):
        v, r, e = packet.read4ByteTxRx(port, mid, address)
        check(r, e, f'Read ID={mid}, address={address}')
        return int(v if v < 2**31 else v - 2**32)

    def write4(mid, address, v):
        r, e = packet.write4ByteTxRx(port, mid, address, int(v))
        check(r, e, f'Write ID={mid}, address={address}')

    def read_positions():
        return np.array([read4(mid, ADDR_PRESENT) for mid in MOTOR_IDS], dtype=int)

    def check_limits(p):
        for mid, value in zip(MOTOR_IDS, p):
            lo, hi = MOTOR_LIMITS[mid]
            if not lo <= int(value) <= hi:
                raise RuntimeError(f'Motor {mid} position {value} outside [{lo}, {hi}]')

    if not port.openPort():
        raise RuntimeError(f'Cannot open {PORT}. Close any other program using it.')
    try:
        if not port.setBaudRate(BAUD):
            raise RuntimeError(f'Cannot set baud rate {BAUD}')
        robot = create_robot()
        robot.tool = SE3.Tz(TCP_OFFSET)
        start = read_positions()
        check_limits(start)
        q_start = motor_to_joint(start)
        xyz_start = np.asarray(robot.fkine(q_start).t, dtype=float).reshape(3)
        xyz_goal = requested if mode == 'absolute' else xyz_start + requested / 1000.0
        move_m = float(np.linalg.norm(xyz_goal - xyz_start))
        if move_m < 0.001:
            raise RuntimeError("Movement must be at least 1 mm")
        solution = robot.ikine_LM(
            SE3(*xyz_goal), q0=q_start,
            mask=[1, 1, 1, 0, 0, 0], joint_limits=True,
        )
        if not solution.success or solution.q is None:
            raise RuntimeError(f'Inverse kinematics failed: {solution.reason}')
        q_goal = np.asarray(solution.q, dtype=float).copy()
        q_goal[4] = q_start[4]  # Never send a new target to motor 5
        goal = np.asarray(joint_to_motor(q_goal), dtype=int)
        goal[4] = start[4]
        check_limits(goal)
        steps = goal - start
        if int(np.max(np.abs(steps[:4]))) > MAX_MOTOR_STEPS:
            raise RuntimeError(f'Motor motion exceeds {MAX_MOTOR_STEPS} steps: {steps.tolist()}')
        model_goal = np.asarray(robot.fkine(motor_to_joint(goal)).t, dtype=float).reshape(3)
        residual = float(np.linalg.norm(model_goal - xyz_goal))
        if residual > MAX_IK_RESIDUAL_M:
            raise RuntimeError(f'IK/rounding residual {residual*1000:.2f} mm exceeds 2 mm')
        # Sample encoder-rounded line in motor space. Cannot detect collisions of links/cables.
        n = max(20, int(np.max(np.abs(steps[:4]))) + 1)
        min_z = float('inf')
        for a in np.linspace(0, 1, n):
            p = np.rint(start + a * steps).astype(int)
            check_limits(p)
            z = float(robot.fkine(motor_to_joint(p)).t[2])
            min_z = min(min_z, z)
        if min_z < MIN_TCP_Z_DH_M:
            raise RuntimeError(f'Model TCP base-Z {min_z*1000:.1f} mm < 150 mm guard')

        print('\n========= SLOW CARTESIAN IK =========', flush=True)
        print('Mode:', 'EXECUTE (MANUAL CONFIRMATION REQUIRED)' if args.execute else 'READ-ONLY DRY RUN')
        print('Coordinates: DH base frame, NOT tabletop frame')
        print('Current motor positions: ', start.tolist())
        print('Current model TCP [mm]:', np.round(xyz_start*1000, 2))
        print('Requested TCP [mm]:   ', np.round(xyz_goal*1000, 2))
        print('Predicted goal TCP [mm]:', np.round(model_goal*1000, 2))
        print('Move XYZ [mm]:', np.round((xyz_goal - xyz_start)*1000, 2))
        print('Motor target: ', goal.tolist())
        print('Motor changes:', steps.tolist())
        print('Model error [mm]:', round(residual*1000, 3))
        print('Minimum sampled TCP DH-Z [mm]:', round(min_z*1000, 2))
        print('Motor velocity profile:', PROFILE_VELOCITY, '| acceleration:', PROFILE_ACCELERATION)
        print('WARNING: No physical collision checking. Motor writes are sequential, not synchronized.')
        print('WARNING: Motor 3 has shown severe following errors. No automatic correction.')
        print('====================================\n', flush=True)
        if not args.execute:
            print('DRY RUN: No motor writes; no movement.')
            return

        for mid in MOTOR_IDS:
            expected_mode = 3 if mid == 3 else 3
            if read1(mid, ADDR_MODE) != expected_mode:
                raise RuntimeError(f'ID {mid}: wrong operating mode')
            if read1(mid, ADDR_HW_ERROR) != 0:
                raise RuntimeError(f'ID {mid}: hardware error')
            if read1(mid, ADDR_TORQUE) != 1:
                raise RuntimeError(f'ID {mid}: torque not enabled; refusing to enable automatically')
        print('Use a real hardware emergency stop; Ctrl+C will NOT stop a motor already commanded.')
        print('Verify all link/pen/cable paths and keep people clear; this plan does NOT certify safety.')
        phrase = input('Type MOVE ONCE to send this ONE slow goal (anything else cancels): ')
        if phrase != 'MOVE ONCE':
            print('Cancelled: no motor commands sent.')
            return
        latest = read_positions()
        if np.max(np.abs(latest - start)) > 2:
            raise RuntimeError(f'Robot moved since planning: {latest.tolist()}. Re-run.')
        for mid in MOTOR_IDS[:4]:
            write4(mid, ADDR_ACCEL, PROFILE_ACCELERATION)
            write4(mid, ADDR_VELOCITY, PROFILE_VELOCITY)
        for mid, pos in zip(MOTOR_IDS[:4], goal[:4]):
            write4(mid, ADDR_GOAL, pos)
        print('Single slow target sent to IDs 1-4. ID 5 unchanged.', flush=True)
        for k in range(20):
            time.sleep(0.25)
            actual = read_positions()
            err = actual - goal
            print(f'{(k+1)*0.25:4.2f}s | pos={actual.tolist()} | goal_error={err.tolist()}', flush=True)
            for mid in MOTOR_IDS:
                if read1(mid, ADDR_HW_ERROR):
                    print(f'HARDWARE ERROR ID {mid}: no additional goal commands will be sent.', flush=True)
                    return
        actual = read_positions()
        xyz_actual = np.asarray(robot.fkine(motor_to_joint(actual)).t, dtype=float).reshape(3)
        print('\n========= FINAL RESULT =========')
        print('Final motor positions:', actual.tolist())
        print('Motor error [steps]: ', (actual - goal).tolist())
        print('Estimated TCP [mm]: ', np.round(xyz_actual*1000, 2))
        print('Estimated movement [mm]: ', np.round((xyz_actual-xyz_start)*1000, 2))
        print('Target error XYZ [mm]:', np.round((xyz_goal-xyz_actual)*1000, 2))
        print('Target 3D error [mm]:', round(np.linalg.norm(xyz_goal-xyz_actual)*1000, 2))
        print('Model estimate from encoder, NOT external physical TCP measurement.')
        print('No automatic retries or further motor commands.')
    finally:
        port.closePort()

if __name__ == '__main__':
    main()
