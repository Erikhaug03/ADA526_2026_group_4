#!/usr/bin/env python3
"""Read-only ROS2 check of DH frame points and tool tip along a proposed IK path.

No Dynamixel writes; does NOT validate physical link geometry or guarantee
collision-free motion. Uses the same current-position/IK planner as
move_to_point_safe.py and samples every interpolated motor step.
"""
import argparse
import csv
from pathlib import Path
import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray
from rosify_dynamixel_ros_pkg.motor_config import motor_to_joint
from rosify_dynamixel_ros_pkg.move_to_point_safe import MoveToPointSafe


class CheckRobotPath(MoveToPointSafe):
    def __init__(self, estimated_base_height_mm=None, clearance_mm=20.0):
        # Parent opens port for reads; execute=False hard locks this node to read-only.
        super().__init__(execute=False)
        self.estimated_base_height_mm = estimated_base_height_mm
        self.clearance_mm = float(clearance_mm)
        self.get_logger().info('PATH CHECK READ-ONLY: No motion or torque commands')
        if estimated_base_height_mm is None:
            self.get_logger().warning('Base-to-plate height unknown: reporting DH-frame Z only')
        else:
            self.get_logger().warning(
                f'Using UNVERIFIED DH-origin height {estimated_base_height_mm:.1f} mm above plate; '
                '59 mm measured to motor top is NOT proof of DH-origin height')

    def frame_points(self, q):
        """Return model frame origins, plus TCP, all in the robot base frame."""
        # Standard DH Link.A(q_i) yields each local transformation.
        from spatialmath import SE3
        transform = SE3()
        result = [('base', np.asarray(transform.t, dtype=float).reshape(3))]
        for i, (link, angle) in enumerate(zip(self.robot.links, q), start=1):
            transform = transform * link.A(float(angle))
            result.append((f'frame_{i}', np.asarray(transform.t, dtype=float).reshape(3)))
        tcp = np.asarray(self.robot.fkine(q).t, dtype=float).reshape(3)
        result.append(('pen_tip_TCP', tcp))
        return result

    @staticmethod
    def motor_samples(start, waypoints):
        """Subdivide every step between waypoints to max 1 encoder step/joint."""
        prev = np.asarray(start, dtype=int)
        yield 0, prev.copy()
        index = 0
        for wp in waypoints:
            wp = np.asarray(wp, dtype=int)
            n = max(1, int(np.max(np.abs(wp[:4] - prev[:4]))))
            for k in range(1, n + 1):
                position = np.rint(prev + (wp - prev) * (k / n)).astype(int)
                position[4] = start[4]
                index += 1
                yield index, position
            prev = wp

    def on_target(self, msg):
        if self.busy:
            self.get_logger().warning('Already checking a path')
            return
        self.busy = True
        try:
            if len(msg.data) != 3:
                raise ValueError('Target message must contain exactly [x,y,z] meters')
            target = np.asarray(msg.data, dtype=float)
            if not np.all(np.isfinite(target)):
                raise ValueError('Invalid non-finite coordinates')
            current, goal, _, _, _, err, waypoints = self.plan(target)
            samples = list(self.motor_samples(current, waypoints))
            names = [name for name, _ in self.frame_points(motor_to_joint(current))]
            min_z = {name: float('inf') for name in names}
            max_z = {name: -float('inf') for name in names}
            min_step = {name: 0 for name in names}
            sample_rows = []
            tip_at_start = tip_at_end = None
            for number, motor in samples:
                joint_angles = motor_to_joint(motor)
                pts = self.frame_points(joint_angles)
                d = dict(pts)
                tip = d['pen_tip_TCP']
                if number == 0:
                    tip_at_start = tip.copy()
                tip_at_end = tip.copy()
                for name, xyz in pts:
                    z = float(xyz[2])
                    if z < min_z[name]:
                        min_z[name] = z
                        min_step[name] = number
                    max_z[name] = max(max_z[name], z)
                    # Geometry is modeled by frame origins, not solid link volume.
                    sample_rows.append((number, name, *[float(x) * 1000 for x in xyz]))
            print('\n======== READ-ONLY PATH GEOMETRY ========')
            print('Target TCP XYZ [m]:  ', np.round(target, 5))
            print('Motor start:         ', current.tolist())
            print('Motor goal:          ', goal.tolist())
            print('Signed delta:        ', (goal-current).tolist())
            print('Planner waypoints:   ', len(waypoints))
            print('Position samples:    ', len(samples), '(start included; <=1 motor step per sample)')
            print('Start TCP XYZ [mm]: ', np.round(tip_at_start * 1000, 2))
            print('End TCP XYZ [mm]:   ', np.round(tip_at_end * 1000, 2))
            print('Model IK residual [mm]: ', round(err * 1000, 4))
            print('\nDH frame/TCP minimum and maximum base-Z [mm]:')
            print(f'{"Point":16} {"Min Z":>10} {"Max Z":>10} {"At sample":>11}')
            for name in names:
                print(f'{name:16} {min_z[name]*1000:10.2f} {max_z[name]*1000:10.2f} {min_step[name]:11d}')
            if self.estimated_base_height_mm is None:
                print('\nPLATE CLEARANCE: UNKNOWN (no verified DH-origin height above plate).')
            else:
                h = self.estimated_base_height_mm
                print(f'\nASSUMED plate clearance using base height {h:.1f} mm:')
                for name in names:
                    clearance = min_z[name] * 1000 + h
                    print(f'  {name:16}: estimated min {clearance:8.2f} mm above plate')
                monitored = [name for name in names if name != 'base']
                close = [name for name in monitored
                         if min_z[name]*1000 + h < self.clearance_mm]
                if close:
                    print('WARNING: Frame-origin or TCP point below chosen nominal clearance: ' + ', '.join(close))
                else:
                    print('No sampled DH frame origin or TCP is below the chosen nominal clearance.')
                print('NOT a physical safety PASS: no link radius, motor housings, cables,')
                print('tool body, joints, table obstacles, or uncertainty margin modeled.')
            output = Path('/tmp/robot_path_points.csv')
            with output.open('w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['sample', 'point', 'x_mm_base', 'y_mm_base', 'z_mm_base'])
                writer.writerows(sample_rows)
            print(f'\nSaved sampled positions: {output}')
            print('READ ONLY: zero Dynamixel writes; no torque changes.')
            print('=========================================\n', flush=True)
        except Exception as exc:
            self.get_logger().error(f'Path check failed: {exc}')
        finally:
            self.busy = False


def main(args=None):
    parser = argparse.ArgumentParser(description='Read-only DH frame/pen path sampling')
    parser.add_argument('--estimated-base-height-mm', type=float, default=None,
                        help='UNVERIFIED DH base origin above plate in mm (optional).')
    parser.add_argument('--clearance-mm', type=float, default=20.0,
                        help='Illustrative lower threshold for frame origins/TCP only.')
    options, ros_args = parser.parse_known_args(args)
    if options.estimated_base_height_mm is not None and not np.isfinite(options.estimated_base_height_mm):
        parser.error('base height must be finite')
    if not np.isfinite(options.clearance_mm) or options.clearance_mm < 0:
        parser.error('clearance must be nonnegative and finite')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = CheckRobotPath(options.estimated_base_height_mm, options.clearance_mm)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
