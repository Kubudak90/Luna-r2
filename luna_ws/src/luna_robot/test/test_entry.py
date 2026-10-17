"""Shutdown behaviour of entry.run() without ROS: a fake rclpy is injected and entry.py is loaded fresh against it,
so the test works whether or not the real rclpy is installed and leaves sys.modules as it found it.
SIGINTs are real signals sent to this process, the way a terminal Ctrl-C and ros2 launch deliver them. The fake
does not model rclpy's own C-level SIGINT handler; spin() ends the way it does when that handler has shut the
context down (ExternalShutdownException, or a publish that failed on the dead context)."""
import importlib.util
import os
import signal
import sys
import time
import types
import unittest
from unittest import mock

ENTRY = os.path.join(os.path.dirname(__file__), '..', 'luna_robot', 'entry.py')


class FakeRclpy:
    """spin_error: what spin() ends with. sigint_in: the calls during which a SIGINT arrives."""

    def __init__(self, spin_error=None, ok=False, sigint_in=()):
        self.calls, self._ok, self.sigint_in = [], ok, set(sigint_in)
        self.module = types.ModuleType('rclpy')
        self.executors = types.ModuleType('rclpy.executors')
        self.executors.ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
        self.spin_error = spin_error or self.executors.ExternalShutdownException()
        self.module.executors = self.executors
        self.module.init = self.init
        self.module.spin = self.spin
        self.module.ok = self.ok
        self.module.try_shutdown = lambda: self.calls.append('try_shutdown')

    def _step(self, name):
        self.calls.append(name)
        if name in self.sigint_in:
            try:
                os.kill(os.getpid(), signal.SIGINT)
                time.sleep(0.2)     # a KeyboardInterrupt, if any, surfaces here
            except KeyboardInterrupt:
                self.calls.append(f'{name}:interrupted')
                raise

    def init(self):
        self.sigint_handler_at_init = signal.getsignal(signal.SIGINT)
        self.calls.append('init')

    def spin(self, node):
        self._step('spin')
        raise self.spin_error

    def ok(self):
        self._step('ok')
        return self._ok

    def node_cls(self):
        fake = self

        class Node:
            def destroy_node(self):
                fake._step('destroy_node')
        return Node

    def run(self):
        with mock.patch.dict(sys.modules, {'rclpy': self.module, 'rclpy.executors': self.executors}):
            spec = importlib.util.spec_from_file_location('entry_under_test', ENTRY)
            entry = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(entry)
        entry.run(self.node_cls())


class RunShutdown(unittest.TestCase):
    def setUp(self):
        self.saved = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, signal.default_int_handler)   # what a node started from a terminal has

    def tearDown(self):
        signal.signal(signal.SIGINT, self.saved)

    def run_quietly(self, fake):
        try:
            fake.run()
        except KeyboardInterrupt:
            self.fail(f'KeyboardInterrupt escaped run(); calls: {fake.calls}')

    def test_python_keyboardinterrupt_is_off_before_rclpy_installs_its_handler(self):
        fake = FakeRclpy()
        self.run_quietly(fake)
        self.assertIs(fake.sigint_handler_at_init, signal.SIG_IGN)

    def test_ctrl_c_twice_never_raises_keyboardinterrupt(self):
        # terminal Ctrl-C during spin, launch's forwarded SIGINT during cleanup
        fake = FakeRclpy(sigint_in={'spin', 'destroy_node'})
        self.run_quietly(fake)
        self.assertEqual(fake.calls, ['init', 'spin', 'destroy_node', 'try_shutdown'])

    def test_sigint_while_judging_a_spin_error_is_harmless(self):
        # dock_node, Docker 2026-10-17: a timer publish raced the shutdown (RCLError out of spin), then the
        # KeyboardInterrupt surfaced while run() was still deciding whether that error was real
        fake = FakeRclpy(RuntimeError("publisher's context is invalid"), sigint_in={'ok'})
        self.run_quietly(fake)
        self.assertEqual(fake.calls, ['init', 'spin', 'ok', 'destroy_node', 'try_shutdown'])

    def test_sigint_stays_ignored_through_interpreter_teardown(self):
        # A Python-level handler is reset to SIG_DFL when the interpreter finalizes, while rclpy/DDS objects are
        # still being torn down; launch's SIGINT landing then killed nodes with -2 (Docker, 2026-10-17).
        # SIG_IGN is left alone by the finalizer.
        fake = FakeRclpy(sigint_in={'spin'})
        self.run_quietly(fake)
        self.assertIs(signal.getsignal(signal.SIGINT), signal.SIG_IGN)

    def test_runtime_error_still_raises_and_cleans_up(self):
        fake = FakeRclpy(RuntimeError('sensor exploded'), ok=True)
        with self.assertRaisesRegex(RuntimeError, 'sensor exploded'):
            fake.run()
        self.assertEqual(fake.calls[-2:], ['destroy_node', 'try_shutdown'])

    def test_error_raced_by_shutdown_is_swallowed(self):
        fake = FakeRclpy(RuntimeError("publisher's context is invalid"))
        self.run_quietly(fake)
        self.assertEqual(fake.calls[-2:], ['destroy_node', 'try_shutdown'])


if __name__ == '__main__':
    unittest.main()
