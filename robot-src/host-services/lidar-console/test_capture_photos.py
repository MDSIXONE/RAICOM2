import tempfile
import unittest
from pathlib import Path
from capture_photos import capture_batch, next_number


class FakeFrames:
    def __init__(self):
        self.deadlines = []
        self.now = 0.0

    def clock(self):
        return self.now

    def sleep(self, delay):
        self.now += delay

    def frame_after(self, earliest):
        stamp = max(earliest, self.now)
        self.deadlines.append(stamp)
        return stamp, 1000 + stamp, b'\xff\xd8test\xff\xd9'


class CapturePhotosTest(unittest.TestCase):
    def test_two_batches_continue_without_overwriting(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            camera = FakeFrames()
            self.assertEqual(capture_batch(directory, camera, 100, 0.4, camera.clock, camera.sleep), (0, 99))
            original = (directory / '99.jpg').read_bytes()
            self.assertEqual(capture_batch(directory, camera, 100, 0.4, camera.clock, camera.sleep), (100, 199))
            self.assertEqual(len(list(directory.glob('*.jpg'))), 200)
            self.assertEqual((directory / '99.jpg').read_bytes(), original)
            for deadlines in (camera.deadlines[:100], camera.deadlines[100:]):
                self.assertTrue(all(b-a >= 0.4-1e-7 for a, b in zip(deadlines, deadlines[1:])))

    def test_restart_after_incomplete_batch_continues_after_highest_number(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            (directory / '0.jpg').write_bytes(b'existing')
            (directory / '3.jpg').write_bytes(b'existing')
            (directory / 'readme.txt').write_text('unrelated')
            self.assertEqual(next_number(directory), 4)
            camera = FakeFrames()
            self.assertEqual(capture_batch(directory, camera, 2, clock=camera.clock, sleep=camera.sleep), (4, 5))
            self.assertEqual((directory / '3.jpg').read_bytes(), b'existing')


if __name__ == '__main__':
    unittest.main()
