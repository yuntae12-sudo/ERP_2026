#!/usr/bin/env python3
"""Bounded, feedback-only observer for an already-running ROS Master.

No publisher, service client, parameter mutation, node launch, or Serial access.
Operator phase labels describe observations, never authorize vehicle actions.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import time
from urllib.parse import urlparse
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-uri', required=True, help='existing Master URI; no Master is started')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--label', default='observe_current_state', help='operator supplied phase description')
    args = parser.parse_args()
    uri = urlparse(args.master_uri)
    if uri.scheme != 'http' or not uri.hostname or not uri.port:
        parser.error('Require explicit http://host:port Master URI')
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 600:
        parser.error('--seconds must be finite and in [1,600]')
    directory = Path(tempfile.mkdtemp(prefix='erp42-feedback-', dir='/tmp'))
    os.environ['ROS_MASTER_URI'] = args.master_uri
    os.environ['ROS_HOME'] = str(directory / 'ros-home')
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros-log')
    master = xmlrpc.client.ServerProxy(args.master_uri)
    topic = '/erp42_serial/feedback'
    code, message, types = master.getTopicTypes('/erp42_feedback_readonly_preflight')
    if code != 1 or dict(types).get(topic) != 'Total_msgs/SerialFeedBack':
        raise SystemExit('Refusing: expected existing feedback topic/type not present')
    initial_state = master.getSystemState('/erp42_feedback_readonly_preflight')[2]
    import rospy
    from Total_msgs.msg import SerialFeedBack
    expected_md5 = '977be1b73fb4d99913310d68e9241255'
    if SerialFeedBack._md5sum != expected_md5:
        raise SystemExit('Refusing: message MD5 differs from repository contract')
    rospy.init_node('erp42_feedback_observer', anonymous=True, disable_rosout=True, disable_signals=True)
    samples = []
    lock = threading.Lock()
    output = (directory / 'feedback.jsonl').open('x')

    def receive(msg):
        row = dict(received_utc=datetime.now(timezone.utc).isoformat(),
                   received_monotonic=time.monotonic(), phase=args.label,
                   fields={name: getattr(msg, name) for name in msg.__slots__})
        with lock:
            samples.append(row)
            output.write(json.dumps(row) + '\n')
            output.flush()

    subscriber = rospy.Subscriber(topic, SerialFeedBack, receive, queue_size=100)
    started = time.monotonic()
    try:
        state = master.getSystemState(rospy.get_name())[2]
        own_pubs = [name for name, nodes in state[0] if rospy.get_name() in nodes]
        own_subs = [name for name, nodes in state[1] if rospy.get_name() in nodes]
        assert own_pubs == [], own_pubs
        assert own_subs == [topic], own_subs
        while time.monotonic() - started < args.seconds and not rospy.is_shutdown():
            time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.unregister()
        rospy.signal_shutdown('read-only observation complete')
        with lock:
            output.close()
            data = list(samples)
        fields = [s['fields'] for s in data]
        intervals = [b['received_monotonic'] - a['received_monotonic'] for a, b in zip(data, data[1:])]
        alive_steps = [(b['alive'] - a['alive']) % 256 for a, b in zip(fields, fields[1:])]
        enc_steps = [b['encoder'] - a['encoder'] for a, b in zip(fields, fields[1:])]
        summary = dict(directory=str(directory), label=args.label, master_uri=args.master_uri,
                       node=rospy.get_name(), feedback_md5=expected_md5, samples=len(data),
                       elapsed_seconds=time.monotonic() - started,
                       arrival_hz=(len(data)-1)/(data[-1]['received_monotonic']-data[0]['received_monotonic']) if len(data)>1 else None,
                       max_arrival_gap_seconds=max(intervals) if intervals else None,
                       median_arrival_gap_seconds=statistics.median(intervals) if intervals else None,
                       distinct_values={name: sorted({s[name] for s in fields}) for name in ('MorA','EStop','Gear','speed','steer','brake')},
                       alive_step_counts=dict(Counter(alive_steps)),
                       encoder_first=fields[0]['encoder'] if fields else None,
                       encoder_last=fields[-1]['encoder'] if fields else None,
                       encoder_changed_intervals=sum(step != 0 for step in enc_steps),
                       initial_master_state=initial_state,
                       observer_published_topics=[], observer_subscribed_topics=[topic],
                       limits=['Arrival times only: feedback has no source timestamp.',
                               'Existing legacy decoder uses integer speed/steer division; zero speed does not prove physical standstill.',
                               'Alive progression proves observed frame variation, not a vehicle-side watchdog.',
                               'Feedback brake is not proof of equivalent Auto command semantics or brake force.',
                               'Phase label is operator text, not an independently verified physical condition.'])
        (directory / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        if not data:
            raise SystemExit('No feedback received; no hardware conclusion possible')


if __name__ == '__main__':
    main()
