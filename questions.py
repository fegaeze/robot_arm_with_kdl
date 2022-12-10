from PyKDL import *
from math import pi
import matplotlib.pyplot as plt


# For backwards compatibility with older versions of PyKDL that do not provide Joint.Fixed yet?
# Joint.None cannot be used in Python 3 because None is not a valid attribute name.
try:
    Joint.Fixed
except AttributeError:
    Joint.Fixed = getattr(Joint, 'None')


def draw_link(frame_1, frame_2):
    """
        Draws the link that connects two frames on the currently active pyplot
        axis.

        Args:
        - frame_1: KDL frame
        - frame_2: KDL frame

        Returns:
        --

        Raises:
        --
    """
    plt.plot([frame_1.p.x(), frame_2.p.x()], [frame_1.p.y(), frame_2.p.y()],
             'k-o', markersize=10)


def draw_frame(frame, length=100):
    """
        Draws the frame's coordinate system on the currently active pyplot axis.
        The x and y axis are drawn in red (X) and green (Y).

        A frame's coordinate system origin is located on the frame's x, y
        coordinate and rotated around that point with the frame's rotation,
        with respect to the global frame. Each axis has <length> as length.

        Args:
        - frame: KDL frame
        - length: The desired length of axis

        Returns:
        --

        Raises:
        --
    """
    horizontal, vertical = Vector(length, 0, 0), Vector(0, length, 0)

    horizontal_x_axis, horizontal_y_axis, _ = frame.M * horizontal
    vertical_x_axis, vertical_y_axis, _ = frame.M * vertical

    plt.plot([frame.p.x(), frame.p.x() + horizontal_x_axis],
             [frame.p.y(), frame.p.y() + horizontal_y_axis], 'r-o',
             [frame.p.x(), frame.p.x() + vertical_x_axis],
             [frame.p.y(), frame.p.y() + vertical_y_axis], 'g-o')


def create_robot(link_lengths, box_size=None):
    """
        Creates a 2D robot containing only revolute joints using KDL. The
        end-effector of the robot is a revolute joint and is carrying a box.
        The box is grasped in the middle of its top edge.

        Args:
        - link_lengths: an iterable of numbers containing the length of the
            links
        - box_size: a 2-tuple containing the size of the box. If None, no box is
            added to the robot.

        Returns:
        - A KDL kinematic chain.

        Raises:
        --
    """
    robot_chain = Chain()

    for link in link_lengths:
        current_frame = Frame(Rotation.RPY(0, 0, 0), Vector(0, link, 0))
        robot_chain.addSegment(Segment(Joint(Joint.RotZ), current_frame))

    # End Effector
    robot_chain.addSegment(Segment(Joint(Joint.RotZ)))

    if box_size:
        box_chain = Chain()
        length, height = max(box_size), min(box_size)

        box_sides = (
            Segment(Joint(Joint.Fixed), Frame(Vector(length//2, 0.0, 0.0))),
            Segment(Joint(Joint.Fixed), Frame(Vector(0.0, -height, 0.0))),
            Segment(Joint(Joint.Fixed), Frame(Vector(-length, 0.0, 0.0))),
            Segment(Joint(Joint.Fixed), Frame(Vector(0.0, height, 0.0))),
            Segment(Joint(Joint.Fixed), Frame(Vector(length//2, 0.0, 0.0))),
        )

        for side in box_sides:
            box_chain.addSegment(side)

        robot_chain.addChain(box_chain)

    return robot_chain


def draw_kinematic_chain(q, chain, plot_segment_frames=True):
    """
        Draws the kinematic chain <chain> in a specific joint configuration <q>
        on the currently active pyplot axis.

        Args:
        - q: KDL joint array of the robot
        - chain: KDL kinematic chain
        - plot_segment_frames: if True draw the frame of each of the segments

        Returns:
        --

        Raises:
        --
    """
    target_frame_1 = Frame()
    target_frame_2 = Frame()
    fk_solver = ChainFkSolverPos_recursive(chain)

    for i in range(chain.getNrOfSegments()):
        fk_solver.JntToCart(q, target_frame_1, i)
        fk_solver.JntToCart(q, target_frame_2, i + 1)
        draw_link(target_frame_1, target_frame_2)

        if plot_segment_frames:
            draw_frame(target_frame_1)


robot = create_robot([600, 400])
robot_with_box = create_robot([600, 400], [600, 400])

base_frame = Frame()
target_frame = Frame(Rotation.RPY(0.0, 0.0, 0.0), Vector(-400, 400, 0.0))

q_init = JntArray(3)
q_desired = JntArray(3)

for idx, _ in enumerate(q_init):
    q_init[idx] = pi/4

iv_solver = ChainIkSolverVel_pinv(robot)
fk_solver = ChainFkSolverPos_recursive(robot)
ik_solver = ChainIkSolverPos_NR(robot, fk_solver, iv_solver)
ik_solver.CartToJnt(q_init, target_frame, q_desired)


# Plot the robot pose
fig = plt.figure()
draw_kinematic_chain(q_desired, robot_with_box)
draw_frame(base_frame)
draw_frame(target_frame)
plt.xlabel('X (mm)')
plt.ylabel('Y (mm)')
plt.grid()
plt.axis('equal')
plt.show()