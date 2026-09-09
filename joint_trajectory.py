"""Position and velocity interpolation for an already validated joint path."""
from bisect import bisect_right


class JointTrajectory:
    """Shape-preserving cubic Hermite interpolation with stationary endpoints.

    Interior tangents use weighted harmonic means. Monotone segments remain
    inside their recorded position bounds; plateaus and reversals get zero
    tangents. Position and velocity are continuous, acceleration need not be.
    ``times`` must start at zero and increase strictly; positions must have a
    consistent number of joints. Input validation belongs to the caller.
    """

    def __init__(self, times, positions):
        self.times = times
        self.positions = positions
        self.duration = times[-1]
        self.joint_count = len(positions[0])
        self.velocities = [[0.0] * self.joint_count for _ in times]
        intervals = [b - a for a, b in zip(times, times[1:])]
        for i in range(1, len(times) - 1):
            before, after = intervals[i - 1], intervals[i]
            for joint in range(self.joint_count):
                left = (positions[i][joint] - positions[i - 1][joint]) / before
                right = (positions[i + 1][joint] - positions[i][joint]) / after
                if left == 0.0 or right == 0.0 or (left > 0) != (right > 0):
                    continue
                w1, w2 = 2 * after + before, after + 2 * before
                self.velocities[i][joint] = (w1 + w2) / (w1 / left + w2 / right)

    def sample(self, elapsed):
        if elapsed <= 0.0:
            return list(self.positions[0]), [0.0] * self.joint_count
        if elapsed >= self.duration:
            return list(self.positions[-1]), [0.0] * self.joint_count
        i = bisect_right(self.times, elapsed) - 1
        duration = self.times[i + 1] - self.times[i]
        s = (elapsed - self.times[i]) / duration
        q, dq = [], []
        for joint in range(self.joint_count):
            start, end = self.positions[i][joint], self.positions[i + 1][joint]
            v0, v1 = self.velocities[i][joint], self.velocities[i + 1][joint]
            # Relative coordinates avoid cancellation when a joint is stationary.
            delta = end - start
            q.append(start + (3 * s**2 - 2 * s**3) * delta
                     + (s**3 - 2 * s**2 + s) * duration * v0
                     + (s**3 - s**2) * duration * v1)
            dq.append((6 * s - 6 * s**2) * delta / duration
                      + (3 * s**2 - 4 * s + 1) * v0
                      + (3 * s**2 - 2 * s) * v1)
        return q, dq
