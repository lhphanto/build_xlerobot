"""Camera-check replay on November24 frames (contact_graspnet env, ~2 min, no hardware): python grasp/test_look.py

At the hover, descent and lifts the single projected seed used to land on the arm and give false CHECKs. Skips if the
frames are not in samples/grasp/."""
import json, os, subprocess, sys, unittest

HERE = os.path.dirname(os.path.abspath(__file__)); SAMPLES = os.path.join(os.path.dirname(HERE), 'samples', 'grasp')


def look(frame, expect, gripper, bottom=None):
    path = os.path.join(SAMPLES, f'robot-camera-{frame}.png')
    if not os.path.exists(path):
        raise unittest.SkipTest(f'{path} not present')
    cmd = [sys.executable, os.path.join(HERE, 'look.py'), '--image', path, '--object', *map(str, expect), '--gripper', *map(str, gripper), '--label', 'test']
    if bottom is not None:
        cmd += ['--expect-bottom-z', str(bottom)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    return json.loads([l for l in r.stdout.splitlines() if l.startswith('{')][-1])


class Replay(unittest.TestCase):
    def test_hover_with_arm_in_front_finds_the_box(self):     # was CHECK (desk sheet segmented)
        self.assertEqual(look('1795573867701506000', [0.333, -0.158, -0.054], [0.357, -0.173, 0.063])['verdict'], 'OK')

    def test_box_hidden_at_the_grasp_point_is_unseen(self):   # was CHECK
        self.assertEqual(look('1795574866681510000', [0.318, -0.182, -0.053], [0.341, -0.193, 0.014])['verdict'], 'UNSEEN')

    def test_lifted_box_hangs_straight_down(self):            # was CHECK; the vertical-hang model puts it right
        g = [0.342, -0.192, 0.133]; hang = 0.115
        d = look('1795575911635667000', [g[0], g[1], g[2] - 0.6 * hang], g, g[2] - hang)
        self.assertEqual(d['verdict'], 'OK', d)


if __name__ == '__main__':
    unittest.main()
