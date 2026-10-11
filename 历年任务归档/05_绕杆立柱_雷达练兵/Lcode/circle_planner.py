"""环绕飞行航点生成器 — 纯函数，不依赖飞控/雷达对象，方便独立单元测试。"""
import math


def generate_circle_waypoints(center_x, center_y, cur_x, cur_y,
                               radius=0.5, n_points=6, direction="cw", z=1.5):
    """生成围绕(center_x, center_y)半径radius的环绕航点列表(世界系[x,y,z])。

    起始点取"当前位置->圆心连线"与圆的交点(离当前位置最近的圆上一点)，避免
    先横穿到圆上任意一点。direction="cw"(顺时针,顶视)/"ccw"(逆时针,顶视)。
    返回n_points+1个点：绕满一圈(n_points个等分点)后再重复第一个点，确保
    闭合>=360度(否则n_points个点首尾不重合，实际只转了(n_points-1)/n_points圈)。
    """
    if n_points < 3:
        raise ValueError("n_points must be >= 3")
    if direction not in ("cw", "ccw"):
        raise ValueError("direction must be 'cw' or 'ccw'")

    dx = cur_x - center_x
    dy = cur_y - center_y
    dist = math.hypot(dx, dy)
    start_angle = math.atan2(dy, dx) if dist > 1e-6 else 0.0

    sign = -1.0 if direction == "cw" else 1.0
    step = sign * (2 * math.pi / n_points)

    points = []
    for i in range(n_points):
        angle = start_angle + step * i
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        points.append([x, y, z])
    points.append(list(points[0]))
    return points


def compute_detour_waypoint(pole_x, pole_y, cur_x, cur_y, target_x, target_y,
                             safety_radius, margin=0.15):
    """如果从(cur_x,cur_y)直飞(target_x,target_y)的直线会进入以(pole_x,pole_y)为
    圆心、safety_radius为半径的安全区，返回一个绕开安全区的中间航点[x,y]；否则
    返回None(直飞已经安全，不需要绕行)。

    算法：取线段上离杆塔最近的点，把这个点沿"杆塔中心->最近点"方向推到安全半径
    (+margin)之外——不是严格的切线最短路径，是简化近似，但对安全半径远小于航段
    长度的场景跟真正最短路径差距通常只有几厘米，实现和测试都简单得多。
    """
    dx = target_x - cur_x
    dy = target_y - cur_y
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-6:
        return None

    ux, uy = dx / seg_len, dy / seg_len
    to_pole_x, to_pole_y = pole_x - cur_x, pole_y - cur_y
    t = to_pole_x * ux + to_pole_y * uy
    t = max(0.0, min(seg_len, t))
    closest_x = cur_x + ux * t
    closest_y = cur_y + uy * t

    dist_to_pole = math.hypot(closest_x - pole_x, closest_y - pole_y)
    if dist_to_pole >= safety_radius:
        return None

    if dist_to_pole < 1e-6:
        # 最近点恰好落在杆塔中心(退化情况)，任取一个垂直于航线的方向推出
        push_x, push_y = -uy, ux
    else:
        push_x = (closest_x - pole_x) / dist_to_pole
        push_y = (closest_y - pole_y) / dist_to_pole

    detour_radius = safety_radius + margin
    return [pole_x + push_x * detour_radius, pole_y + push_y * detour_radius]
