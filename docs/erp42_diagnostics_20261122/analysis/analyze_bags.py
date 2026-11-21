#!/usr/bin/env python3
"""Offline ERP42 bag analysis; never connects to ROS or publishes commands.

Run after: source /opt/ros/noetic/setup.bash
  /usr/bin/python3 analyze_bags.py

The user timed 14.2 m starting from rest (no separate acceleration lead-in).
For runs 1-5 only, first RX EStop is an approximate arrival marker. Run 6
was stopped later and is excluded from timed-window RX calibration.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

TRIALS = [
    ('23-36-05', 70, 13.82),
    ('23-37-53', 70, 14.34),
    ('23-38-58', 100, 9.83),
    ('23-40-01', 100, 9.26),
    ('23-41-03', 150, 6.45),
    ('23-41-46', 150, 6.24),
]
DISTANCE_M = 14.2
PREFIX = '/erp42_serial/'


def integral(t, y, lo, hi):
    """Trapezoidal integration with interpolated endpoints (no extrapolation)."""
    if not t[0] <= lo <= hi <= t[-1]:
        raise ValueError('Integration bounds outside recorded data')
    knots = np.r_[lo, t[(t > lo) & (t < hi)], hi]
    return float(np.trapz(np.interp(knots, t, y), knots))


def state_key(m):
    return (m.MorA, m.EStop, m.Gear, m.brake, m.encoder, m.alive)


def decode_frame(m):
    data = bytes(m.data)
    if len(data) != 18 or data[:3] != b'STX' or data[-2:] != b'\r\n':
        raise ValueError('Invalid recorded RX frame')
    state = (data[3], data[4], data[5], data[10],
             int.from_bytes(data[11:15], 'little', signed=True), data[15])
    return state, int.from_bytes(data[6:8], 'little'), \
        int.from_bytes(data[8:10], 'little', signed=True)


def match_topic(raw, other, byte_frames=False):
    """Match identical frame metadata within 20ms, independently of speed."""
    times = np.array([t for t, _ in other])
    used, matches, mismatches, offsets = set(), [], 0, []
    for t, m in raw:
        candidates = []
        for i in np.flatnonzero(abs(times - t) < .02):
            if i in used:
                continue
            n = other[i][1]
            state, speed, steer = decode_frame(n) if byte_frames else \
                (state_key(n), n.speed, n.steer)
            if state == state_key(m):
                candidates.append((int(i), speed, steer))
        if not candidates:
            matches.append(None)
            continue
        i, speed, steer = min(candidates, key=lambda item: abs(times[item[0]] - t))
        expected = (m.speed, m.steer) if byte_frames else \
            (math.trunc(m.speed / 10), math.trunc(m.steer / 71))
        mismatches += int((speed, steer) != expected)
        used.add(i)
        offsets.append(abs(times[i] - t))
        matches.append(i)
    return matches, {
        'matched': len(used), 'unmatched_raw': matches.count(None),
        'extra_other_messages': len(other) - len(used),
        'value_mismatches': mismatches,
        'max_receipt_time_difference_ms': max(offsets, default=0.) * 1000,
    }


def previous_value(series, t, field):
    times = [x[0] for x in series]
    i = int(np.searchsorted(times, t, side='right')) - 1
    return getattr(series[i][1], field) if i >= 0 else None


def write_csv(path, rows):
    with path.open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def analyze_trial(base, index, tag, command, duration):
    path = base / ('erp42_speed_check_2026-11-21-' + tag + '.bag')
    topics = {}
    with rosbag.Bag(str(path)) as bag:
        epoch = bag.get_start_time()
        total_time = bag.get_end_time() - epoch
        for topic, msg, stamp in bag.read_messages():
            name = topic[len(PREFIX):] if topic.startswith(PREFIX) else topic
            topics.setdefault(name, []).append((stamp.to_sec() - epoch, msg))
    raw = topics['feedback_raw']
    t = np.array([time for time, _ in raw])
    speed = np.array([m.speed for _, m in raw])
    encoder = np.array([m.encoder for _, m in raw])
    good = np.array([m.MorA == 1 and m.EStop == 0 and m.Gear == 0 for _, m in raw])
    cmd_on = next(time for time, m in topics['command'] if m.KPH == command)
    drive_on = next(time for time, m in topics['drive'] if m.KPH == command)
    stop = next(time for time, m in raw if time > cmd_on and m.EStop == 1)
    active = (t >= cmd_on) & (t < stop) & good
    if not np.all(good[(t >= cmd_on) & (t < stop)]):
        raise ValueError('Unexpected mode interruption during automatic trial')
    feedback_ids, feedback_check = match_topic(raw, topics['feedback'])
    byte_ids, bytes_check = match_topic(raw, topics['feedback_bytes'], True)
    if any(x['unmatched_raw'] or x['value_mismatches'] for x in (feedback_check, bytes_check)):
        raise ValueError('Unexpected mismatch in raw/decoded/converted messages')
    approximate_window = index < 6
    lo = stop - duration if approximate_window else None
    hi = stop if approximate_window else None
    recent = active & (t >= stop - 1)
    last_three = active & (t >= stop - 3)
    raw_area = integral(t, speed, lo, hi) if approximate_window else None
    dt = np.diff(t)
    summary = {
        'run': index, 'bag': path.name, 'command_raw': command,
        'distance_m': DISTANCE_M, 'measured_duration_s': duration,
        'measured_average_kmh': DISTANCE_M / duration * 3.6,
        'command_per_measured_average_kmh': command / (DISTANCE_M / duration * 3.6),
        'bag_duration_s': total_time, 'raw_sample_count': len(raw),
        'raw_rate_hz': (len(t) - 1) / (t[-1] - t[0]),
        'raw_max_gap_s': float(dt.max()),
        'drive_on_bag_s': drive_on, 'command_on_bag_s': cmd_on,
        'drive_to_command_step_s': cmd_on - drive_on, 'estop_bag_s': stop,
        'positive_speed_delay_after_command_s': float(t[active & (speed > 0)][0] - cmd_on),
        'pre_estop_last_1s_raw_mean': float(speed[recent].mean()),
        'pre_estop_last_3s_raw_slope_per_s': float(np.polyfit(t[last_three], speed[last_three], 1)[0]),
        'automatic_run_peak_raw': float(speed[active].max()),
        'approx_timed_window_usable': approximate_window,
        'approx_window_start_bag_s': lo, 'approx_window_end_bag_s': hi,
        'approx_window_raw_integral_count_s': raw_area,
        'approx_window_raw_time_mean': raw_area / duration if approximate_window else None,
        'approx_window_distance_if_divisor_10_m': raw_area / 36 if approximate_window else None,
        'approx_rx_divisor_counts_per_kmh': raw_area / (3.6 * DISTANCE_M) if approximate_window else None,
        'approx_window_encoder_counts_abs': abs(float(np.interp(hi, t, encoder) - np.interp(lo, t, encoder))) if approximate_window else None,
        'command_to_estop_distance_if_divisor_10_m': integral(t, speed, cmd_on, stop) / 36,
    }
    for fraction in (.5, .9):
        hits = np.flatnonzero(active & (speed >= fraction * command))
        summary['first_raw_reaches_{}pct_after_command_s'.format(int(fraction * 100))] = \
            float(t[hits[0]] - cmd_on) if len(hits) else None
    sensitivity = []
    if approximate_window:
        for delay in (0, .25, .5, 1.):
            left, right = lo - delay, hi - delay
            if left >= t[0]:
                sensitivity.append({
                    'assumed_arrival_before_estop_s': delay,
                    'approx_rx_divisor_counts_per_kmh': integral(t, speed, left, right) / (3.6 * DISTANCE_M),
                })
    command_values = sorted(set(m.KPH for time, m in topics['command'] if cmd_on <= time < stop))
    drive_values = sorted(set(m.KPH for time, m in topics['drive'] if cmd_on <= time < stop))
    if command_values != [command] or drive_values != [command]:
        raise ValueError('Input command was not constant during automatic trial')
    details = {
        'bag_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'bag_start_epoch_s': epoch,
        'topic_message_counts': {PREFIX + k: len(v) for k, v in topics.items()},
        'raw_to_feedback_check': feedback_check,
        'bytes_to_raw_check': bytes_check,
        'automatic_command_values': command_values,
        'automatic_drive_values': drive_values,
        'arrival_timing_sensitivity': sensitivity,
    }
    samples = []
    for i, (time, m) in enumerate(raw):
        phase = 'manual' if m.MorA != 1 else 'estop' if m.EStop else \
            'auto_forward' if m.Gear == 0 else 'auto_other_gear'
        samples.append({
            'run': index, 'bag': path.name, 'bag_time_s': time,
            'time_from_command_step_s': time - cmd_on,
            'MorA': m.MorA, 'EStop': m.EStop, 'Gear': m.Gear, 'phase': phase,
            'drive_raw_last_received': previous_value(topics['drive'], time, 'KPH'),
            'command_raw_last_received': previous_value(topics['command'], time, 'KPH'),
            'speed_raw': m.speed,
            'feedback_speed_recorded': topics['feedback'][feedback_ids[i]][1].speed,
            'raw_speed_div_10_float': m.speed / 10.,
            'steer_raw': m.steer,
            'feedback_steer_recorded': topics['feedback'][feedback_ids[i]][1].steer,
            'brake': m.brake, 'encoder': m.encoder, 'alive': m.alive,
            'rx_bytes_hex': bytes(topics['feedback_bytes'][byte_ids[i]][1].data).hex(' '),
            'automatic_trial_sample': bool(active[i]),
            'inside_approx_timed_window': bool(lo <= time <= hi) if approximate_window else False,
        })
    return summary, details, samples, topics


def set_plot_style():
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        plt.rcParams['font.family'] = FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'figure.facecolor': 'white'})


def plot_response(output, results, fit):
    set_plot_style()
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    colors = ('#1766a6', '#dd6c20')
    for ax, value in zip(axes.flat, (70, 100, 150)):
        group = [r for r in results if r[0]['command_raw'] == value]
        for repetition, (s, _, _, topics) in enumerate(group, 1):
            selected = [(t - s['command_on_bag_s'], m.speed) for t, m in topics['feedback_raw']
                        if s['command_on_bag_s'] <= t < s['estop_bag_s']
                        and m.MorA == 1 and m.EStop == 0 and m.Gear == 0]
            x, y = np.array(selected).T
            ax.plot(x, y, color=colors[repetition - 1], linewidth=1.6,
                    label='시험 {}: 수신 원시값'.format(s['run']))
            ax.plot(x[-1], y[-1], 's', color=colors[repetition - 1], markersize=5)
        ax.axhline(value, color='#454545', ls='--', lw=1.5, label='고정 명령 {}'.format(value))
        ax.set(title='명령 {}: 두 차례의 수신 속도'.format(value),
               xlabel='명령이 /command에 나타난 뒤 시간 (s)', ylabel='속도 원시값', ylim=(0, value * 1.12), xlim=(0, None))
        ax.grid(alpha=.2)
        ax.legend(fontsize=9, loc='lower right')
    ax = axes[1, 1]
    x = np.array([r[0]['command_raw'] for r in results])
    y = np.array([r[0]['measured_average_kmh'] for r in results])
    line_x = np.linspace(min(x), max(x), 100)
    ax.plot(line_x, line_x * fit['through_origin_slope'], color='#454545', ls='--',
            label='평균 속도 ≈ 입력 / {:.2f}'.format(fit['through_origin_divisor']))
    for i, (xx, yy) in enumerate(zip(x, y)):
        ax.scatter(xx, yy, s=55, color=colors[i % 2], zorder=3)
        ax.annotate(str(i + 1), (xx, yy), xytext=(-18, 4) if i % 2 == 0 else (9, 4),
                    textcoords='offset points', fontsize=9)
    ax.set(title='14.2m 정지 출발 시험의 실측 평균 속도', xlabel='고정 입력 원시값',
           ylabel='14.2 / 측정시간 × 3.6 (km/h)', xlim=(55, 165), ylim=(2.5, 9))
    ax.grid(alpha=.2)
    ax.legend(fontsize=9, loc='upper left')
    ax.text(.04, .07, '가속 구간을 포함한 평균값\n일정 속도 주행의 보정식으로 사용할 수 없음',
            transform=ax.transAxes, fontsize=10, color='#8c3a16')
    fig.suptitle('ERP42: 명령은 일정하지만 수신 속도는 계속 상승', fontsize=17, y=.985)
    fig.text(.5, .02, '곡선의 끝은 EStop 직전. 시험 6은 도착 후 추가 이동 포함. 수신값의 상승만으로 실제 속도와 내부 필터 지연을 분리할 수는 없음.',
             ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .05, 1, .95))
    fig.savefig(str(output / 'speed_response.png'), dpi=180)
    fig.savefig(str(output / 'speed_response.svg'))
    plt.close(fig)


def plot_timelines(output, results):
    fig, axes = plt.subplots(3, 2, figsize=(14, 11))
    for ax, (s, _, _, topics) in zip(axes.flat, results):
        raw = topics['feedback_raw']
        t = np.array([t for t, m in raw])
        y = np.array([m.speed for t, m in raw])
        manual = np.array([m.MorA == 0 for t, m in raw])
        estop = np.array([m.MorA == 1 and m.EStop == 1 for t, m in raw])
        lim = s['command_raw'] * 1.12
        ax.fill_between(t, 0, lim, where=estop, step='post', color='#f7d3d3', alpha=.8)
        ax.fill_between(t, 0, lim, where=manual, step='post', color='#dedede', alpha=.9)
        if s['approx_timed_window_usable']:
            ax.axvspan(s['approx_window_start_bag_s'], s['approx_window_end_bag_s'],
                       color='#d8ebfb', alpha=.45)
        ct = [t for t, m in topics['command']]
        cy = [m.KPH for t, m in topics['command']]
        ax.step(ct, cy, where='post', color='#454545', ls='--', linewidth=1.3)
        ax.plot(t, y, color='#1766a6', linewidth=1.5)
        ft = [t for t, m in topics['feedback']]
        fy = [m.speed * 10 for t, m in topics['feedback']]
        ax.step(ft, fy, where='post', color='#dd6c20', linewidth=.8, alpha=.7)
        ax.set(title='시험 {} · {} · 명령 {} · {:.2f}s'.format(
            s['run'], s['bag'][-12:-4], s['command_raw'], s['measured_duration_s']),
            xlabel='bag 시작 후 시간 (s)', ylabel='원시 단위 (feedback은 ×10)',
            ylim=(0, lim), xlim=(0, s['bag_duration_s']))
        ax.grid(alpha=.17)
    handles = [Line2D([], [], color='#454545', ls='--', label='/command.KPH'),
               Line2D([], [], color='#1766a6', label='/feedback_raw.speed'),
               Line2D([], [], color='#dd6c20', label='/feedback.speed × 10'),
               Patch(facecolor='#d8ebfb', label='측정 구간 근사: EStop 시각 − 측정시간'),
               Patch(facecolor='#f7d3d3', label='EStop 구간: 분석 제외'),
               Patch(facecolor='#dedede', label='수동 구간: 분석 제외')]
    fig.suptitle('전체 기록과 분석에서 제외한 구간', fontsize=17, y=.99)
    fig.legend(handles=handles, loc='lower center', ncol=3, fontsize=9)
    fig.tight_layout(rect=(0, .065, 1, .96))
    fig.savefig(str(output / 'bag_timelines.png'), dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag-dir', type=Path, default=Path.home())
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = [analyze_trial(args.bag_dir, i, *trial) for i, trial in enumerate(TRIALS, 1)]
    summaries = [r[0] for r in results]
    x = np.array([s['command_raw'] for s in summaries], dtype=float)
    y = np.array([s['measured_average_kmh'] for s in summaries])
    slope = float(x @ y / (x @ x))
    affine = np.polyfit(x, y, 1)
    fit = {
        'scope': 'Only mean speed over these six 14.2 m starts from rest; not a steady-state or RX-unit calibration.',
        'through_origin_slope': slope, 'through_origin_divisor': 1 / slope,
        'through_origin_centered_r_squared': float(1 - np.sum((y - slope * x)**2) / np.sum((y - y.mean())**2)),
        'affine_slope': float(affine[0]), 'affine_intercept_kmh': float(affine[1]),
        'affine_r_squared': float(1 - np.sum((y - np.polyval(affine, x))**2) / np.sum((y - y.mean())**2)),
    }
    report = {
        'assumptions_and_limits': [
            'User confirmed these six tests start from rest; previous 1.42 m run-in does not apply.',
            'Elapsed times and 14.2 m distance supplied by user; no exact start/finish markers in bags.',
            'For runs 1-5, arrival is approximated by first feedback_raw EStop=1, then measured duration is subtracted.',
            'Run 6 EStop was later than arrival: no RX-to-distance calibration is computed for run 6.',
            'All bag times are receipt times relative to each bag start, not sensor acquisition timestamps.',
            'Manual, EStop and non-forward states excluded from automatic-run metrics.',
            'RX divisors are provisional and confounded by manual timing and stop reaction delay.',
            'A rising RX speed does not by itself separate physical acceleration from internal telemetry filtering.',
            'The command topic confirms the host command value, not electrical delivery or acknowledgement by the vehicle.',
            'No driver settings, calibration constants, ROS nodes or vehicle commands were changed by this analysis.',
        ],
        'formulae': {
            'measured_average_kmh': '3.6 * distance_m / measured_duration_s',
            'provisional_rx_divisor': 'integral(raw_speed dt) / (3.6 * distance_m)',
            'distance_if_divisor_10_m': 'integral(raw_speed dt) / 36',
            'recorded_feedback_speed': 'trunc(raw_speed / 10)',
            'recorded_feedback_steer': 'trunc(raw_steer / 71)',
        },
        'command_to_measured_average_fit': fit,
        'trials': [dict(summary=s, **detail) for s, detail, _, _ in results],
    }
    write_csv(args.output_dir / 'speed_summary.csv', summaries)
    write_csv(args.output_dir / 'speed_samples.csv', [row for _, _, rows, _ in results for row in rows])
    with (args.output_dir / 'speed_analysis.json').open('w', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    plot_response(args.output_dir, results, fit)
    plot_timelines(args.output_dir, results)
    print('run command time_s mean_kmh raw_mean_est rx_divisor_est distance_div10_est')
    for s in summaries:
        print(s['run'], s['command_raw'], s['measured_duration_s'],
              round(s['measured_average_kmh'], 4),
              *[round(s[k], 4) if s[k] is not None else 'N/A' for k in (
                  'approx_window_raw_time_mean', 'approx_rx_divisor_counts_per_kmh',
                  'approx_window_distance_if_divisor_10_m')])
    print('Mean-speed fit only:', json.dumps(fit))
    print('Validated raw samples:', sum(s['raw_sample_count'] for s in summaries))
    print('Output:', args.output_dir)


if __name__ == '__main__':
    main()
