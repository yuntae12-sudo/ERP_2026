#!/usr/bin/env python3
"""ROS/PTY integration only. Run with --workspace from run_offline.py.

Linux bubblewrap is mandatory; fail before launching ROS if its device/PID
namespace isolation cannot be demonstrated. All protocol values are synthetic.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty
import xmlrpc.client


def sandbox(directory, share_host_pts=True):
    # New /dev and /proc hide hardware and host process descriptors. The only
    # host character devices exposed are PTYs. No inherited device FDs/caps.
    command = ['bwrap', '--unshare-all', '--share-net', '--die-with-parent', '--as-pid-1',
            '--new-session', '--ro-bind', '/', '/', '--proc', '/proc',
            '--dev', '/dev']
    if share_host_pts:
        command += ['--dev-bind', '/dev/pts', '/dev/pts']
    return command + [
            '--bind', str(directory), str(directory), '--cap-drop', 'ALL']


class Mock:
    def __init__(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.running = True
        self.feedback = True
        self.corrupt = False
        self.split = False
        self.mode_override = None  # Mock physical selector/absent mode acknowledgement.
        self.alive = 0
        self.encoder = -217
        self.packets = []
        self.sent = []
        self.last = bytes.fromhex('5354580001010000000025000d0a')
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buffer = b''
        due = time.monotonic()
        while self.running:
            try:
                if select.select([self.master], [], [], .003)[0]:
                    data = os.read(self.master, 4096)
                    buffer += data
                    while len(buffer) >= 14:
                        p, buffer = buffer[:14], buffer[14:]
                        if p[:3] != b'STX' or p[-2:] != b'\r\n':
                            raise RuntimeError('Invalid command framing: ' + p.hex())
                        self.last = p
                        self.packets.append((time.monotonic(), p))
                now = time.monotonic()
                if self.feedback and now >= due:
                    due = now + .02
                    p = self.last
                    speed = int.from_bytes(p[6:8], 'big') if p[4] == 0 and p[10] == 0 else 0
                    steer = int.from_bytes(p[8:10], 'big', signed=True)
                    if speed:
                        self.encoder += 1
                    self.alive = (self.alive + 1) % 256
                    mode_bytes = p[3:6] if self.mode_override is None else bytes([self.mode_override]) + p[4:6]
                    frame = (b'STX' + mode_bytes + struct.pack('<HhBiB', speed, steer,
                             p[10], self.encoder, self.alive) + b'\r\n')
                    if self.corrupt:
                        frame = b'BAD' + frame[3:]
                    self.sent.append((now, frame))
                    if self.split:
                        os.write(self.master, frame[:7])
                        time.sleep(.05)  # Deliberately exceed the 20ms partial deadline.
                        os.write(self.master, frame[7:])
                    else:
                        os.write(self.master, frame)
            except OSError:
                if self.running:
                    time.sleep(.003)

    def close(self):
        self.running = False
        self.thread.join(1)
        os.close(self.master)
        os.close(self.slave)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    args = parser.parse_args()
    ws = args.workspace.resolve()
    root = Path(__file__).resolve().parents[3]
    if subprocess.check_output(['git', '--no-optional-locks', 'branch', '--show-current'], cwd=root, text=True).strip() != 'test/erp42-low-speed-drive':
        raise SystemExit('Wrong work branch')
    if not str(ws).startswith('/tmp/erp42-safety-') or not (ws / 'devel/lib/erp42_serial/erp42_serial_node').is_file():
        raise SystemExit('Expected a separately built run_offline.py workspace')
    if os.environ.get('ERP42_INTEGRATION_ENV') != str(ws):
        env = dict(os.environ)
        env.update(ERP42_INTEGRATION_ENV=str(ws), CMAKE_PREFIX_PATH=str(ws / 'devel') + ':/opt/ros/noetic',
                   ROS_PACKAGE_PATH=str(ws / 'src') + ':/opt/ros/noetic/share',
                   PYTHONPATH=str(ws / 'devel/lib/python3/dist-packages') + ':/opt/ros/noetic/lib/python3/dist-packages',
                   LD_LIBRARY_PATH=str(ws / 'devel/lib') + ':/opt/ros/noetic/lib', PYTHONDONTWRITEBYTECODE='1')
        os.execve('/usr/bin/python3', ['/usr/bin/python3', str(Path(__file__).resolve()), '--workspace', str(ws)], env)
    directory = Path(tempfile.mkdtemp(prefix='erp42-ros-integration-', dir='/tmp'))
    print('Integration directory:', directory, flush=True)
    proof = subprocess.run(sandbox(directory) + ['/usr/bin/python3', '-c',
        'import os,json; assert not any(n.startswith(("ttyUSB","ttyACM")) for n in os.listdir("/dev")); '
        'assert not os.path.exists("/proc/5082/fd"); '
        'print(json.dumps({"dev":os.listdir("/dev"),"pid_namespace":os.readlink("/proc/self/ns/pid"),"caps":open("/proc/self/status").read().split("CapEff:")[1].splitlines()[0].strip()}))'],
        capture_output=True, text=True, close_fds=True)
    (directory / 'isolation-probe.log').write_text(proof.stdout + proof.stderr)
    if proof.returncode:
        raise SystemExit('BLOCKED: OS isolation unavailable; no ROS processes launched')
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    assert port != 11311
    uri = 'http://127.0.0.1:' + str(port)
    env = dict(os.environ, ROS_MASTER_URI=uri, ROS_IP='127.0.0.1',
               ROS_HOME=str(directory / 'ros-home'), ROS_LOG_DIR=str(directory / 'ros-log'))
    env.pop('ROS_HOSTNAME', None)
    os.environ.update(env)
    import rospy
    from Total_msgs.msg import DriveCmd, ModeCmd, CmdControl, SerialFeedBack
    from std_srvs.srv import Trigger
    results = {}
    command = []
    feedback = []
    services = []
    node_audit = []
    transition_steps = []
    auxiliary_processes = []
    nodes = []
    mocks = []
    logs = []
    master = None
    active_case = 1
    current = None
    mock = None
    publisher_running = True
    send_drive = False
    send_mode = False
    drive = (0, 0, 37)
    mode = (1, 0, 1, 0)
    master_api = xmlrpc.client.ServerProxy(uri)

    def record(number, evidence):
        results['ROS-%03d' % number] = dict(result='PASS', evidence=evidence)
        print('PASS ROS-%03d: %s' % (number, evidence), flush=True)

    def wait(predicate, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError('Condition did not occur within %.2fs' % timeout)

    def latest():
        return command[-1][1] if command else None

    def locked():
        p = latest()
        return p is not None and p.KPH == 0 and p.brake == 37 and p.EStop == 1

    def signal_test_node(process, sig):
        path = Path('/proc/%d' % process.node_pid)
        # Guard against PID reuse; never signal a PID merely remembered by name.
        assert os.readlink(path / 'exe') == str(ws / 'devel/lib/erp42_serial/erp42_serial_node')
        assert (path / 'stat').read_text().split()[21] == process.node_start
        os.kill(process.node_pid, sig)

    def stop_node(sig=signal.SIGINT):
        if current and current.poll() is None:
            # Signal the proven child PID, not bubblewrap (which would SIGKILL
            # its child with --die-with-parent and bypass ROS graceful exit).
            signal_test_node(current, sig)
            try:
                current.wait(3)
            except subprocess.TimeoutExpired:
                current.kill()
                current.wait(3)

    def start_node(confirmed=True, overrides=None, expect_open=True):
        nonlocal current, mock
        mock = Mock()
        mocks.append(mock)
        base = '/erp42_serial/erp42_serial_node'
        config = dict(serial_port=mock.path, serial_baudrate=115200, protocol_confirmed=confirmed,
                      allow_hardware_serial=False, stop_brake=37, brake_min=0, brake_max=255,
                      brake_release=0, estop_assert=1, estop_release=0, speed_limit_raw=10,
                      steer_limit_raw=1000, initial_gear=1, allowed_gears=[0, 1, 2],
                      stationary_hold=.3, drive_timeout=.25, mode_timeout=.25, feedback_timeout=.25)
        config.update(manual_transition_confirmed=True, manual_transition_timeout=1.0,
                      stationary_only=False, test_deadline_sec=60.0)
        config.update(overrides or {})
        for key in list(config):
            if config[key] is None:
                master_api.deleteParam('/integration_driver', base + '/' + key)
                del config[key]
        for key, value in config.items():
            master_api.setParam('/integration_driver', base + '/' + key, value)
        log = directory / ('node-%d.log' % len(nodes))
        out = log.open('w')
        logs.append(out)
        info = (directory / ('namespace-%d.json' % len(nodes))).open('w+')
        current = subprocess.Popen(sandbox(directory) + ['--info-fd', str(info.fileno()), '/usr/bin/stdbuf', '-oL', '-eL', str(ws / 'devel/lib/erp42_serial/erp42_serial_node'),
                    '__ns:=/erp42_serial', '__name:=erp42_serial_node'], env=env,
                    stdout=out, stderr=subprocess.STDOUT, close_fds=True, pass_fds=(info.fileno(),))
        nodes.append(current)
        if not confirmed or not expect_open:
            current.wait(3)
            info.close()
            assert not mock.packets
            return
        wait(lambda: len(mock.packets) > 2)
        info.seek(0)
        current.node_pid = json.load(info)['child-pid']
        current.node_start = Path('/proc/%d/stat' % current.node_pid).read_text().split()[21]
        info.close()
        dev = Path('/proc/%d/root/dev' % current.node_pid)
        assert not any(p.name.startswith(('ttyUSB', 'ttyACM')) for p in dev.iterdir())
        device_fds = []
        for f in Path('/proc/%d/fd' % current.node_pid).iterdir():
            try:
                target = os.readlink(f)
                if target.startswith('/dev/'):
                    device_fds.append(target)
            except FileNotFoundError:
                pass
        assert mock.path in device_fds and set(device_fds) <= {mock.path, '/dev/null'}, device_fds
        node_audit.append(dict(pid=current.node_pid, wrapper_pid=current.pid, config=config,
                               device_fds=device_fds, namespace=json.loads((directory / ('namespace-%d.json' % (len(nodes) - 1))).read_text())))
        rospy.wait_for_service(base + '/rearm', timeout=3)
        wait(lambda: pubdrive.get_num_connections() == 1 and pubmode.get_num_connections() == 1)
        time.sleep(.12)

    def arm():
        nonlocal drive, mode, send_drive, send_mode
        drive = (0, 0, 37)
        mode = (1, 0, mode[2], 0)
        send_drive = send_mode = True
        time.sleep(.4)
        response = rearm()
        assert response.success, response.message
        wait(lambda: latest() is not None and latest().EStop == 0 and latest().Gear == mode[2])
        time.sleep(.09)
        assert latest().KPH == 0 and latest().brake == 37
        drive = (1, 0, 0)
        wait(lambda: latest().KPH == 1)

    try:
        out = (directory / 'master.log').open('w')
        logs.append(out)
        master = subprocess.Popen(['/opt/ros/noetic/bin/rosmaster', '--core', '-p', str(port)],
                                  env=env, stdout=out, stderr=subprocess.STDOUT, close_fds=True)
        def master_ready():
            try:
                return master_api.getPid('/integration_driver')[0] == 1
            except OSError:
                return False
        wait(master_ready)
        assert master_api.getSystemState('/integration_driver')[2] == [[], [], []]
        record(1, 'New empty master %s, PID %s; subprocess-only environment' % (uri, master.pid))
        rospy.init_node('integration_driver', anonymous=False, disable_signals=True)
        pubdrive = rospy.Publisher('/erp42_serial/drive', DriveCmd, queue_size=1)
        pubmode = rospy.Publisher('/erp42_serial/mode', ModeCmd, queue_size=1)
        rospy.Subscriber('/erp42_serial/command', CmdControl, lambda m: command.append((time.monotonic(), m)))
        rospy.Subscriber('/erp42_serial/feedback', SerialFeedBack, lambda m: feedback.append((time.monotonic(), m)))
        trigger = rospy.ServiceProxy('/erp42_serial/erp42_serial_node/rearm', Trigger)
        def rearm():
            response = trigger()
            services.append(dict(time=time.monotonic(), service='/erp42_serial/erp42_serial_node/rearm',
                                 success=response.success, message=response.message))
            return response
        def publish():
            while publisher_running:
                if send_mode:
                    pubmode.publish(ModeCmd(*mode))
                if send_drive:
                    pubdrive.publish(DriveCmd(*drive))
                time.sleep(.03)
        publisher = threading.Thread(target=publish, daemon=True)
        publisher.start()
        active_case = 2
        start_node(confirmed=False)
        start_node()
        assert not rearm().success  # No Mode/Drive commands have been published.
        send_drive = send_mode = True
        registered = master_api.getSystemState('/integration_driver')[2]
        assert all(n in ('/integration_driver', '/erp42_serial/erp42_serial_node')
                   for section in registered for topic, names in section for n in names)
        record(2, 'Bubblewrap new /dev,/proc; cap-drop ALL; PTY %s; unconfirmed protocol exited without writes' % mock.path)
        active_case = 3
        expected = ['69a82c1e3ba067908d8fabb7e7d8716a', '93db1f5d15225b4e964829a0cf6a3bdb',
                    '17043bad9780a7db1065d8002634203b', '977be1b73fb4d99913310d68e9241255']
        assert [c._md5sum for c in (DriveCmd, ModeCmd, CmdControl, SerialFeedBack)] == expected
        record(3, dict(zip(['DriveCmd', 'ModeCmd', 'CmdControl', 'SerialFeedBack'], expected)))
        active_case = 8
        drive = (1, 0, 0)
        time.sleep(.1)
        assert locked()
        record(8, 'ROS DriveCmd KPH=1 at startup -> command KPH=0 brake=37 EStop=1')
        active_case = 9
        assert not rearm().success
        drive = (0, 0, 37)
        time.sleep(.15)
        arm()
        record(9, 'Trigger denied nonzero Drive; fresh zero Drive/Auto/stationary hold permits explicit rearm')
        record(4, 'New ROS DriveCmd after rearm -> captured speed bytes 0001')
        record(5, 'ROS ModeCmd Auto/release gear=1 -> transmitted mode=1 gear=1')
        active_case = 17
        drive = (1, 71, 0)
        wait(lambda: feedback and abs(feedback[-1][1].speed - .1) < 1e-9 and abs(feedback[-1][1].steer - 1) < 1e-9)
        record(17, 'KPH=1 -> bytes6..7=0001; Mock feedback speed=0.1 km/h; Deg=71 -> steer=1 degree')
        # Exact packet comparison, indexed by alive and all fields, not timing.
        def encoded(m):
            return b'STX' + bytes([m.MorA, m.EStop, m.Gear]) + struct.pack('>HhBB', m.KPH, m.Deg, m.brake, m.alive) + b'\r\n'
        time.sleep(.1)
        packets = [p for t, p in mock.packets]
        # Exclude only the most recent in-flight network/PTY samples, and compare
        # every completed sample (not an arbitrary matching subset).
        active_case = 6
        cutoff = time.monotonic() - .05
        completed_commands = [m for t, m in command if t < cutoff]
        matched = [m for m in completed_commands if encoded(m) in packets]
        assert len(matched) == len(completed_commands) and len(matched) >= 10
        record(6, '%d ROS command samples matched exact captured 14-byte packets' % len(matched))
        sent = [p for t, p in mock.sent]
        active_case = 7
        count = 0
        completed_feedback = [f for t, f in feedback if t < cutoff]
        for f in completed_feedback:
            raw = b'STX' + bytes([f.MorA, f.EStop, f.Gear]) + struct.pack('<HhBiB', round(f.speed * 10), round(f.steer * 71), f.brake, f.encoder, f.alive) + b'\r\n'
            count += raw in sent
        assert count == len(completed_feedback) and count >= 10
        record(7, '%d ROS feedback samples matched exact Mock 18-byte frames' % count)
        active_case = 16
        drive = (1, 0, 73)
        wait(lambda: latest().brake == 73 and latest().KPH == 0)
        assert mock.last[10] == 73
        record(16, 'ROS brake=73 -> byte10=73, speed/steer zero (synthetic brake)')
        active_case = 18
        drive = (1, 0, 0)
        wait(lambda: latest().KPH == 1)
        start = time.monotonic()
        time.sleep(1)
        assert latest().KPH == 1  # 0.25s watchdog with 30ms updates has no false latch.
        drive = (0, 0, 37)
        wait(lambda: latest().KPH == 0 and latest().brake == 37)
        record(18, 'Mock-only 1s stream -> explicit zero/brake packet after %.3fs; no false watchdog at 30ms updates' % (time.monotonic() - start))
        for number, channel in ((10, 'drive'), (11, 'mode'), (12, 'feedback')):
            active_case = number
            drive = (1, 0, 0)
            wait(lambda: latest().KPH == 1)
            start = time.monotonic()
            if channel == 'drive':
                send_drive = False
            elif channel == 'mode':
                send_mode = False
            else:
                mock.feedback = False
            wait(locked, .6)
            elapsed = time.monotonic() - start
            assert .18 < elapsed < .5, elapsed
            if number == 12:
                assert not rearm().success
            record(number, '%s stream alone stopped; locked zero/brake/EStop in %.3fs (timeout=.25)' % (channel, elapsed))
            send_drive = send_mode = mock.feedback = True
            drive = (1, 0, 0)
            time.sleep(.12)
            assert locked()
            arm()
        mode = (1, 1, 1, 0)
        active_case = 13
        wait(locked)
        record(13, 'ROS ModeCmd EStop=1 discards active speed=1 -> locked stop packet')
        mode = (1, 0, 1, 0)
        active_case = 14
        time.sleep(.12)
        assert locked() and not rearm().success
        record(14, 'EStop release + continuing nonzero Drive cannot restart; rearm denied')
        arm()
        mode = (1, 0, 2, 0)
        active_case = 15
        wait(locked)
        assert latest().Gear == 1
        record(15, 'Gear change 1->2 latches; old speed discarded and previous applied gear retained pending rearm')
        time.sleep(.1)
        assert locked()
        arm()
        assert latest().Gear == 2
        record(24, 'Fresh zero Drive + stationary Mock + explicit Trigger + new Drive restores motion with requested gear=2')
        # New Manual path, entirely through test-Master Topic/PTY/feedback.
        active_case = 25
        marker = len(mock.packets)
        mode = (0, 0, 2, 0)
        wait(lambda: latest().KPH == 0 and latest().brake == 37)
        wait(lambda: latest().MorA == 0 and latest().EStop == 0)
        wait(lambda: feedback[-1][1].MorA == 0 and feedback[-1][1].speed == 0)
        node_log = directory / ('node-%d.log' % (len(nodes)-1))
        wait(lambda: 'ERP42 safety state 4' in node_log.read_text())
        stopped_auto = [p for t,p in mock.packets[marker:] if p[3] == 1 and p[6:10] == b'\x00'*4 and p[10] == 37]
        assert stopped_auto, 'Manual was requested before a stop in Auto'
        assert 'ERP42 safety state 3' in node_log.read_text()
        record(25, 'Manual ModeCmd: state3 -> stopped Auto/Brake packet -> Manual packet -> new Manual feedback -> logged state4')
        # Existing nonzero stream continues through the transition and after it.
        time.sleep(.12)
        assert latest().MorA == 0 and latest().KPH == 0 and latest().brake == 37
        record(26, 'Continuing speed=1 DriveCmd discarded during ManualTransition and ManualLocked')
        mode = (1, 0, 2, 0)
        wait(locked)
        assert not rearm().success
        time.sleep(.12)
        assert locked()
        record(27, 'Auto ModeCmd alone remains locked; Trigger rejects nonzero Drive; explicit zero/rearm/new Drive required')
        arm()
        assert latest().MorA == 1 and latest().KPH == 1
        record(28, 'Fresh zero Drive + explicit Trigger + new Auto feedback + new Drive re-enters Auto after Manual')
        conflict = (1, 71, 1)
        drive = conflict
        wait(lambda: latest().KPH == 0 and latest().brake == 1 and latest().Deg == 0)
        wait(lambda: 'brake=1 differs from brake_release=0' in node_log.read_text(), 2)
        record(29, 'Legacy ROS Brake=1 against release=0 -> byte10=1, zero speed/steer, explicit contract diagnostic, EStop remains released')
        drive = (1, 0, 0)
        wait(lambda: latest().KPH == 1)
        active_case = 23
        mock.corrupt = True
        wait(locked)
        before = len(feedback)
        time.sleep(.12)
        assert len(feedback) == before
        mock.corrupt = False
        time.sleep(.25)
        arm()
        mock.split = True
        wait(locked)
        before = len(feedback)
        time.sleep(.18)
        assert len(feedback) == before
        record(23, 'BAD header frames not published; 7+11 byte frames beyond 20ms deadline rejected and latch stop')
        mock.split = False
        # Restart after injected framing loss, not guessed stream resync.
        stop_node()
        start_node()
        arm()
        active_case = 19
        before = len(mock.packets)
        stop_node()
        wait(lambda: len(mock.packets) > before)
        last = mock.packets[-1][1]
        assert last[4] == 1 and last[6:10] == b'\x00' * 4 and last[10] == 37
        record(19, 'SIGINT test node: final captured packet zero/stop brake/EStop; best effort only, not physical stop')
        active_case = 20
        start_node()
        arm()
        active_case = 21
        stop_node(signal.SIGKILL)
        time.sleep(.06)
        # Still publishing speed=1 on the test Master during restart.
        start_node()
        assert locked()
        assert not rearm().success
        record(20, 'SIGKILL only test PID, restart with ongoing ROS speed=1 starts Locked; no cached approval; SIGKILL has no stop guarantee')
        arm()
        mock.close()  # PTY master hangup: real POSIX poll/read/write failures.
        wait(lambda: 'Serial write failed' in (directory / ('node-%d.log' % (len(nodes) - 1))).read_text())
        logtext = (directory / ('node-%d.log' % (len(nodes) - 1))).read_text()
        assert 'Invalid/missing Serial frame' in logtext
        count = len(command)
        time.sleep(.15)
        assert len(command) == count
        response = rearm()
        assert not response.success and 'transport recovery required' in response.message
        record(21, 'PTY hangup -> POSIX read/poll error; ROS adapter logs invalid/missing frame, latches motion off')
        active_case = 22
        record(22, 'Same PTY hangup -> write failure; ROS command publishing ceases; Trigger refuses framing-unknown transport')
        # Session constraints through real ROS Topics/Trigger and captured PTY bytes.
        stop_node()
        send_drive = send_mode = True
        drive, mode = (0, 0, 37), (1, 0, 1, 0)
        active_case = 30
        start_node(overrides=dict(stationary_only=True, speed_limit_raw=0, test_deadline_sec=20.0))
        stationary_mock = mock
        def arm_zero():
            nonlocal drive, mode
            drive, mode = (0, 0, 37), (1, 0, 1, 0)
            path = directory / ('node-%d.log' % (len(nodes)-1))
            previous_arms = path.read_text().count('ERP42 safety state 2')
            time.sleep(.4)
            response = rearm()
            assert response.success, response.message
            wait(lambda: path.read_text().count('ERP42 safety state 2') > previous_arms)
            wait(lambda: latest().EStop == 0 and latest().KPH == 0)
        drive = (1, 0, 0)
        node_log = directory / ('node-%d.log' % (len(nodes)-1))
        wait(lambda: 'Stationary-only violation: speed=1' in node_log.read_text())
        assert locked()
        assert all(p[6:10] == b'\x00'*4 for t,p in mock.packets)
        record(30, 'Stationary=true: ROS positive KPH rejected, logged, Locked; every PTY packet speed/steer zero')
        active_case = 31
        drive = (0, -71, 0)
        wait(lambda: 'steer=-71' in node_log.read_text(), 2)
        assert locked() and all(p[6:10] == b'\x00'*4 for t,p in mock.packets)
        record(31, 'ROS nonzero steering rejected with diagnostic; captured 14-byte speed/steer bytes all zero')
        active_case = 32
        arm_zero()
        assert 'ERP42 safety state 1' in node_log.read_text()
        assert 'ERP42 safety state 2' in node_log.read_text()
        assert all(p[6:10] == b'\x00'*4 for t,p in mock.packets)
        record(32, 'Actual Trigger approval, Releasing/Armed logs and PTY zero/zero invariant')
        active_case = 39
        marker = len(mock.packets)
        drive = (1, 71, 0)
        wait(locked)
        assert all(p[6:10] == b'\x00'*4 for t,p in mock.packets[marker:])
        assert not rearm().success
        arm_zero()
        record(39, 'Nonzero ROS Drive while stationary Armed locks/discards motion; fresh zero/rearm still emits only zero/zero')
        active_case = 33
        mode = (1, 1, 1, 0)
        wait(locked)
        arm_zero()
        mode = (0, 0, 1, 0)
        wait(lambda: latest().MorA == 0 and latest().EStop == 0)
        wait(lambda: 'ERP42 safety state 4' in node_log.read_text())
        assert all(p[6:10] == b'\x00'*4 for t,p in mock.packets)
        stop_node()
        assert all(p[6:10] == b'\x00'*4 for t,p in stationary_mock.packets)
        record(33, 'EStop fault, zero rearm, ManualTransition/ManualLocked and graceful shutdown all emit zero/zero')
        active_case = 34
        drive, mode = (0, 0, 37), (1, 0, 1, 0)
        start_node(overrides=dict(stationary_only=True, speed_limit_raw=0, test_deadline_sec=1.5))
        arm_zero()
        node_log = directory / ('node-%d.log' % (len(nodes)-1))
        wait(lambda: 'test deadline expired' in node_log.read_text())
        wait(locked)
        record(34, 'Fresh ROS zero input streams continued; steady session deadline independently enters Locked')
        active_case = 35
        response = rearm()
        assert not response.success and 'deadline' in response.message
        record(35, 'Actual post-deadline Trigger response rejects rearm: ' + response.message)
        active_case = 36
        marker = len(mock.packets)
        drive, mode = (1, 71, 0), (1, 0, 2, 0)
        time.sleep(.15)
        assert locked() and latest().Gear == 1
        assert all(p[6:10] == b'\x00'*4 and p[4] == 1 for t,p in mock.packets[marker:])
        assert not rearm().success
        record(36, 'Post-deadline ROS Drive/Mode and rearm never restore motion or change applied gear')
        stop_node()
        active_case = 37
        # Sim time is changed only on the isolated master. roscpp uses /clock,
        # while the node session clock stays steady_clock and its loop is wall-time.
        master_api.setParam('/integration_driver', '/use_sim_time', True)
        from rosgraph_msgs.msg import Clock
        pubclock = rospy.Publisher('/clock', Clock, queue_size=1)
        drive, mode = (0, 0, 37), (1, 0, 1, 0)
        start_node(overrides=dict(stationary_only=True, speed_limit_raw=0, test_deadline_sec=1.2))
        wait(lambda: pubclock.get_num_connections() >= 1)
        clock_events = []
        for sec in (1000, 2, 5000, 0):
            pubclock.publish(Clock(rospy.Time(sec)))
            clock_events.append(dict(steady=time.monotonic(), clock_sec=sec))
            time.sleep(.08)
        master_api.setParam('/integration_driver', '/use_sim_time', False)
        node_log = directory / ('node-%d.log' % (len(nodes)-1))
        wait(lambda: 'test deadline expired' in node_log.read_text())
        response = rearm()
        assert not response.success and 'deadline' in response.message
        assert locked()
        record(37, dict(evidence='Actual /clock forward/back jumps and isolated /use_sim_time changes did not reset steady deadline', clock_events=clock_events))
        pubclock.unregister()
        stop_node()
        active_case = 38
        start_node(overrides=dict(stationary_only=True, speed_limit_raw=0, test_deadline_sec=None), expect_open=False)
        assert not mock.packets
        record(38, 'Missing mandatory test_deadline_sec with otherwise Mock-valid stationary policy: no PTY open/write')
        # Full stationary mode session and independent failure scenarios.
        def transition_step(label):
            wait(lambda: latest() is not None and feedback)
            transition_steps.append(dict(label=label, steady=time.monotonic(),
                command={k:getattr(latest(),k) for k in latest().__slots__},
                feedback={k:getattr(feedback[-1][1],k) for k in feedback[-1][1].__slots__}))
        def stationary_session(deadline=8.0):
            nonlocal drive, mode, send_drive, send_mode, node_log
            stop_node()
            drive, mode = (0, 0, 37), (1, 0, 1, 0)
            send_drive = send_mode = False
            start_node(overrides=dict(stationary_only=True, speed_limit_raw=0, test_deadline_sec=deadline))
            node_log = directory / ('node-%d.log' % (len(nodes)-1))
            wait(lambda: latest().MorA == 0 and locked() and feedback[-1][1].MorA == 0)
        def begin_auto():
            nonlocal send_drive, send_mode
            send_drive = send_mode = True
            time.sleep(.4)
            response = rearm()
            assert response.success, response.message
        def every_packet_zero():
            assert mock.packets and all(p[6:10] == b'\x00'*4 for t,p in mock.packets)
        active_case = 40
        stationary_session()
        transition_step('startup_Manual_Locked')
        begin_auto()
        wait(lambda: 'ERP42 safety state 2' in node_log.read_text())
        wait(lambda: feedback[-1][1].MorA == 1 and latest().EStop == 0)
        transition_step('Auto_ACK_Armed_zero')
        time.sleep(.12)
        mode = (0, 0, 1, 0)
        wait(lambda: 'ERP42 safety state 4' in node_log.read_text())
        wait(lambda: feedback[-1][1].MorA == 0 and feedback[-1][1].EStop == 0)
        transition_step('Manual_ACK_ManualLocked_zero')
        send_drive = send_mode = False
        wait(locked, .6)
        wait(lambda: feedback[-1][1].MorA == 0 and feedback[-1][1].EStop == 1)
        transition_step('all_inputs_stopped_watchdog_Locked_Manual')
        assert latest().MorA == 0 and latest().Gear == 1
        every_packet_zero()
        stop_node()
        every_packet_zero()
        assert mock.packets[-1][1][3:6] == bytes([0,1,1])
        record(40, 'Full ROS/Trigger/PTY Manual0 Locked -> Auto ACK Armed -> Manual ACK ManualLocked -> input stop -> Watchdog Locked (Manual0); all packets zero; final stop packet verified')
        active_case = 41
        stationary_session(deadline=1.5)
        mock.mode_override = 0
        begin_auto()
        wait(lambda: 'ERP42 safety state 1' in node_log.read_text())
        time.sleep(.15)
        assert 'ERP42 safety state 2' not in node_log.read_text()
        assert feedback[-1][1].MorA == 0 and latest().MorA == 1
        wait(lambda: 'test deadline expired' in node_log.read_text())
        wait(locked)
        assert not rearm().success
        every_packet_zero()
        record(41, 'Advancing stationary Manual feedback never ACKs Auto: remains Releasing until session deadline, then Locked/rearm denied (no dedicated Auto-ACK timer)')
        active_case = 42
        stationary_session()
        send_drive = send_mode = True
        arm_zero()
        mock.mode_override = 1
        mode = (0,0,1,0)
        wait(lambda: 'Manual transition: waiting for post-write Manual feedback' in node_log.read_text())
        wait(lambda: 'Manual transition timeout' in node_log.read_text())
        wait(locked)
        assert 'ERP42 safety state 4' not in node_log.read_text()
        assert latest().MorA == 0 and feedback[-1][1].MorA == 1
        every_packet_zero()
        record(42, 'Fresh Auto feedback without Manual ACK: transition times out, Locked with commanded Manual0 but feedback still Auto1; never falsely reports ManualLocked')
        for number, channel in ((43,'drive'),(44,'mode')):
            active_case = number
            stationary_session()
            send_drive = send_mode = True
            arm_zero()
            mode = (0,0,1,0)
            wait(lambda: 'ERP42 safety state 3' in node_log.read_text())
            if channel == 'drive':
                send_drive = False
            else:
                send_mode = False
            wait(locked, .6)
            assert ('DriveCmd timeout' if channel == 'drive' else 'ModeCmd timeout') in node_log.read_text()
            every_packet_zero()
            record(number, '%s input stopped during stationary Manual return: watchdog faults Locked and keeps zero packet' % channel)
        active_case = 45
        stationary_session()
        send_drive = send_mode = True
        arm_zero()
        mode=(0,1,1,0)
        wait(locked)
        assert 'ERP42 safety state 3' not in node_log.read_text()
        every_packet_zero()
        record(45, 'EStop assertion with Manual request has priority; no Manual recovery shortcut, zero stop packets')
        active_case = 46
        stationary_session()
        send_drive = send_mode = True
        arm_zero()
        # Positive speed + brake=1 was covered by ROS-029/039. Here zero input
        # with a valid non-release brake is not falsely turned into EStop.
        drive=(0,0,1)
        wait(lambda: latest().brake == 1 and latest().EStop == 0)
        every_packet_zero()
        drive=(1,0,1)
        wait(locked)
        wait(lambda: 'Stationary-only violation' in node_log.read_text())
        assert not rearm().success
        every_packet_zero()
        record(46, 'Zero input with non-release brake preserves byte10 without EStop; nonzero legacy brake1 conflict is rejected by higher-priority stationary gate')
        active_case = 47
        stationary_session(deadline=1.2)
        send_drive = send_mode = True
        arm_zero()
        mock.mode_override = 1
        mode=(0,0,1,0)
        wait(lambda: 'ERP42 safety state 3' in node_log.read_text())
        wait(lambda: 'test deadline expired' in node_log.read_text())
        wait(locked)
        assert 'ERP42 safety state 4' not in node_log.read_text()
        assert not rearm().success
        every_packet_zero()
        record(47, 'Whole-session deadline expires during Manual transition; Locked/zero, rearm cannot renew time')
        active_case = 48
        stationary_session()
        send_drive = send_mode = True
        arm_zero()
        publisher_code = """import rospy,time
from Total_msgs.msg import DriveCmd,ModeCmd
rospy.init_node('transition_fault_publisher',disable_signals=True)
a=rospy.Publisher('/erp42_serial/drive',DriveCmd,queue_size=1)
b=rospy.Publisher('/erp42_serial/mode',ModeCmd,queue_size=1)
while not rospy.is_shutdown():
 a.publish(DriveCmd(0,0,37));b.publish(ModeCmd(1,0,1,0));time.sleep(.03)
"""
        aux_info=(directory/'publisher-namespace.json').open('w+')
        aux_log=(directory/'publisher.log').open('w');logs.append(aux_log)
        aux=subprocess.Popen(sandbox(directory)+['--info-fd',str(aux_info.fileno()),'/usr/bin/python3','-B','-c',publisher_code],
            env=env,stdout=aux_log,stderr=subprocess.STDOUT,close_fds=True,pass_fds=(aux_info.fileno(),))
        auxiliary_processes.append(aux)
        def aux_connected():
            state=master_api.getSystemState('/integration_driver')[2]
            return all(any(topic==target and '/transition_fault_publisher' in names for topic,names in state[0])
                       for target in ('/erp42_serial/drive','/erp42_serial/mode'))
        wait(aux_connected)
        time.sleep(.12)
        send_drive=send_mode=False
        pubdrive.unregister();pubmode.unregister()
        time.sleep(.12)
        assert latest().EStop==0
        aux_info.seek(0);aux_pid=json.load(aux_info)['child-pid'];aux_info.close()
        assert Path('/proc/%d/exe'%aux_pid).resolve()==Path('/usr/bin/python3').resolve()
        os.kill(aux_pid,signal.SIGKILL)
        aux.wait(3)
        wait(locked,.6)
        assert 'timeout' in node_log.read_text()
        every_packet_zero()
        record(48, 'Actual isolated ROS publisher child SIGKILL: node survives, missing zero Drive/Mode streams trigger Locked stop packets')


    except Exception as exc:
        import traceback
        (directory / 'failure.log').write_text(traceback.format_exc())
        print('FAILED:', repr(exc), flush=True)
        results['ROS-%03d' % active_case] = dict(result='FAIL', evidence=repr(exc))
        for i in range(1, 49):
            results.setdefault('ROS-%03d' % i, dict(result='BLOCKED', evidence='Earlier failure: ' + repr(exc)))
        # The currently executing case is recorded through failure.log; never claim PASS.
    finally:
        publisher_running = False
        for aux in auxiliary_processes:
            if aux.poll() is None:
                aux.terminate()
                try:
                    aux.wait(3)
                except subprocess.TimeoutExpired:
                    aux.kill(); aux.wait(3)
        for process in reversed(nodes):
            if process.poll() is None:
                if hasattr(process, 'node_pid'):
                    signal_test_node(process, signal.SIGINT)
                else:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(3)
        for m in mocks:
            if m.running:
                m.close()
        if 'rospy' in locals():
            rospy.signal_shutdown('isolated integration finished')
        if master and master.poll() is None:
            master.send_signal(signal.SIGINT)
            try:
                master.wait(3)
            except subprocess.TimeoutExpired:
                master.kill()
                master.wait(3)
        for out in logs:
            out.close()
        (directory / 'captures.json').write_text(json.dumps(dict(
            services=services, transition_steps=transition_steps,
            commands=[dict(time=t, fields={s: getattr(m, s) for s in m.__slots__}) for t, m in command],
            feedback=[dict(time=t, fields={s: getattr(m, s) for s in m.__slots__}) for t, m in feedback],
            mocks=[dict(path=m.path, packets=[dict(time=t, hex=p.hex()) for t, p in m.packets],
                        sent=[dict(time=t, hex=p.hex()) for t, p in m.sent]) for m in mocks]), indent=2))
        (directory / 'integration-report.json').write_text(json.dumps(dict(
            workspace=str(ws), directory=str(directory), master_uri=uri,
            environment={k: env[k] for k in ('ROS_MASTER_URI','ROS_PACKAGE_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','ROS_HOME','ROS_LOG_DIR')},
            isolation_probe=json.loads(proof.stdout), hardware_tested=False,
            binary_sha256=hashlib.sha256((ws / 'devel/lib/erp42_serial/erp42_serial_node').read_bytes()).hexdigest(),
            node_audit=node_audit,
            process_cleanup=all(p.poll() is not None for p in nodes + auxiliary_processes + ([master] if master else [])),
            results=results), indent=2))
        print('Report:', directory / 'integration-report.json', flush=True)
    return 0 if len(results) == 48 and all(v['result'] == 'PASS' for v in results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
