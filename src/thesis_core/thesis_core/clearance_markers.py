"""RViz dimensions for instantaneous and nominal JOG proximity."""
import math

from builtin_interfaces.msg import Duration as DurationMessage
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def clearance_markers(header, name, hit, color, state, *, nominal=False):
    """Draw the measured gap and an unobstructed label for box obstacles."""
    axis = hit.closest_robot_point
    target = hit.box_point if hit.box_point is not None else hit.obstacle_center
    delta = tuple(b - a for a, b in zip(axis, target))
    length = math.hypot(*delta)
    direction = tuple(v / length for v in delta) if length else (0., 0., 1.)
    start = tuple(v + hit.capsule_radius * d for v, d in zip(axis, direction))
    end = target if hit.box_point is not None else tuple(
        v - hit.obstacle_radius * d for v, d in zip(target, direction))

    line = Marker(header=header, ns=name, id=0, type=Marker.LINE_LIST)
    line.pose.orientation.w = 1.
    line.scale.x = .008
    line.color.r, line.color.g, line.color.b, line.color.a = (*color, 1.)
    line.points = [Point(x=x, y=y, z=z) for x, y, z in (start, end)]
    line.lifetime = DurationMessage(nanosec=300000000)
    label_position = [
        (start[0] + end[0]) / 2,
        (start[1] + end[1]) / 2,
        (start[2] + end[2]) / 2 + .055,
    ]
    markers = [line]
    if hit.box_point is not None or nominal:
        # Draw an offset copy with extension lines so a gap hidden by the
        # pedestal can still be read in the side view.
        radius = math.hypot(end[0], end[1])
        lateral = ((end[0] / radius, end[1] / radius) if radius > .04
                   else (.8, -.6))
        offset = (.25 * lateral[0], .25 * lateral[1], .14)
        outer_start, outer_end = [
            Point(x=p[0] + offset[0], y=p[1] + offset[1], z=p[2] + offset[2])
            for p in (start, end)
        ]
        leader = Marker(
            header=header, ns=name, id=2, type=Marker.LINE_LIST,
        )
        leader.pose.orientation.w = 1.
        leader.scale.x = .006
        leader.color = line.color
        leader.points = [
            line.points[1], outer_end, outer_end, outer_start,
            outer_start, line.points[0],
        ]
        leader.lifetime = line.lifetime
        markers.append(leader)
        label_position = [outer_start.x, outer_start.y,
                          max(outer_start.z, outer_end.z) + .055]

    label = Marker(header=header, ns=name, id=1, type=Marker.TEXT_VIEW_FACING)
    label.pose.orientation.w = 1.
    label.pose.position = Point(
        x=label_position[0], y=label_position[1], z=label_position[2],
    )
    label.scale.z = .034
    label.color = line.color
    label.lifetime = line.lifetime
    label.text = f'{name} {hit.clearance:.3f} m' + (f' · {state}' if state else '')
    if nominal:
        obstacle = 'Mesa' if hit.box_point is not None else 'Esfera'
        quantity = 'Solape' if hit.clearance < 0 else 'Distancia'
        label.text = (f'JOG/{obstacle}\n{hit.segment_name}\n'
                      f'{quantity}={abs(hit.clearance):.3f}m\nt=+{hit.sample_time:.2f}s')
    elif hit.box_point is not None:
        label.text = f'{hit.clearance:.3f} m'
    markers.append(label)
    return markers
