import math


def quat_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return quat_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def quat_conjugate(q):
    x, y, z, w = quat_normalize(q)
    return (-x, -y, -z, w)


def rotate_vector(q, vector):
    x, y, z, w = quat_normalize(q)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose_transform(first, second):
    first_t, first_q = first
    second_t, second_q = second
    rotated = rotate_vector(first_q, second_t)
    return (
        tuple(first_t[i] + rotated[i] for i in range(3)),
        quat_multiply(first_q, second_q),
    )


def inverse_transform(transform):
    translation, rotation = transform
    inverse_rotation = quat_conjugate(rotation)
    inverse_translation = rotate_vector(
        inverse_rotation, tuple(-value for value in translation)
    )
    return inverse_translation, inverse_rotation


def yaw_from_quaternion(q):
    x, y, z, w = quat_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

