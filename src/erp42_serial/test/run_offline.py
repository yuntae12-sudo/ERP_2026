#!/usr/bin/env python3
"""Build in a new /tmp workspace; never launch a ROS node or use a vehicle port."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from run_ros_integration import sandbox


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sanitize', action='store_true', help='also compile/run ASan+UBSan core/transport tests')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    branch = subprocess.check_output(['git', '--no-optional-locks', 'branch', '--show-current'], cwd=root, text=True).strip()
    if branch != 'test/erp42-low-speed-drive':
        raise SystemExit('Refusing: expected test/erp42-low-speed-drive branch')
    workspace = Path(tempfile.mkdtemp(prefix='erp42-safety-', dir='/tmp'))
    print('Isolated workspace:', workspace, flush=True)
    (workspace / 'src').mkdir()
    for package in ('Total_msgs', 'erp42_serial'):
        (workspace / 'src' / package).symlink_to(root / 'src' / package, target_is_directory=True)
    env = dict(os.environ)
    env.update(CMAKE_PREFIX_PATH='/opt/ros/noetic', ROS_PACKAGE_PATH='/opt/ros/noetic/share',
               PYTHONPATH='/opt/ros/noetic/lib/python3/dist-packages',
               LD_LIBRARY_PATH='/opt/ros/noetic/lib', PYTHONDONTWRITEBYTECODE='1',
               ROS_HOME=str(workspace / 'ros-home'), ROS_MASTER_URI='http://127.0.0.1:1')
    proof = subprocess.run(sandbox(workspace, share_host_pts=False) + ['/usr/bin/python3', '-c',
        'import os,json; assert not any(n.startswith(("ttyUSB","ttyACM")) for n in os.listdir("/dev")); '
        'assert not os.path.exists("/proc/5082/fd"); '
        'a,b=os.openpty(); print(json.dumps({"devices":os.listdir("/dev"),"pty":os.ttyname(b),"pid_namespace":os.readlink("/proc/self/ns/pid")}))'],
        env=env, capture_output=True, text=True, close_fds=True)
    (workspace / 'isolation-probe.log').write_text(proof.stdout + proof.stderr)
    if proof.returncode:
        raise SystemExit('BLOCKED: device isolation unavailable; tests not run')

    def run(command, log):
        with (workspace / log).open('w') as out:
            result = subprocess.run(command, cwd=workspace, env=env, stdout=out, stderr=subprocess.STDOUT)
        if result.returncode:
            print((workspace / log).read_text())
            raise SystemExit('Failed; see ' + str(workspace / log))
        print('PASS:', log, flush=True)

    run(['/opt/ros/noetic/bin/catkin_make', '-C', str(workspace),
         '-DPYTHON_EXECUTABLE=/usr/bin/python3', '-DCMAKE_BUILD_TYPE=Debug', '-j2'], 'build.log')
    run(['cmake', '--build', str(workspace / 'build'), '--target', 'erp42_safety_test', '-j2'], 'test-build.log')
    env['LD_LIBRARY_PATH'] = ':'.join([str(workspace / 'devel/lib'), str(workspace / 'build/lib'), '/opt/ros/noetic/lib'])
    run(sandbox(workspace, share_host_pts=False) + [str(workspace / 'devel/lib/erp42_serial/erp42_safety_test'),
         '--gtest_output=xml:' + str(workspace / 'safety-results.xml')], 'safety-tests.log')
    if args.sanitize:
        p = root / 'src/erp42_serial'
        run(['g++', '-std=c++11', '-g', '-O1', '-Wall', '-Wextra', '-Werror',
             '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
             '-I', str(p / 'include'), '-I', '/usr/src/googletest/googletest/include',
             '-I', '/usr/src/googletest/googletest', str(p / 'src/safety_control.cpp'),
             str(p / 'src/SerialPort/serial_port.cpp'), str(p / 'test/safety_test.cpp'),
             '/usr/src/googletest/googletest/src/gtest-all.cc', '-pthread', '-lutil',
             '-o', str(workspace / 'safety-sanitized')], 'sanitizer-build.log')
        env['ASAN_OPTIONS'] = 'detect_leaks=1:halt_on_error=1'
        env['UBSAN_OPTIONS'] = 'halt_on_error=1:print_stacktrace=1'
        run(sandbox(workspace, share_host_pts=False) + [str(workspace / 'safety-sanitized'),
             '--gtest_output=xml:' + str(workspace / 'sanitized-results.xml')], 'sanitized-tests.log')
    xml = ET.parse(workspace / 'safety-results.xml')
    cases = []
    for case in xml.iter('testcase'):
        cases.append({'name': case.attrib['name'], 'suite': case.attrib['classname'],
                      'result': 'FAIL' if case.find('failure') is not None else 'PASS',
                      'properties': {p.attrib['name']: p.attrib['value'] for p in case.findall('./properties/property')}})
    report = {'build': 'PASS', 'hardware_tested': False, 'ros_adapter_runtime_tested': False,
              'forced_exit_stop_guaranteed': False, 'workspace': str(workspace),
              'sanitizers': 'PASS' if args.sanitize else 'NOT_RUN', 'tests': cases}
    report['device_isolation'] = json.loads(proof.stdout)
    (workspace / 'offline-report.json').write_text(json.dumps(report, indent=2))
    print('Report:', workspace / 'offline-report.json', flush=True)


if __name__ == '__main__':
    main()
