#!/usr/bin/env python3
"""Build the Korean PDF from recorded analysis, without connecting to ROS.

Dependencies: python3-reportlab, python3-numpy, python3-matplotlib.
Nanum Gothic (SIL OFL) is cached from google/fonts if not already available.
Run: /usr/bin/python3 build_report.py [--font-dir /path/to/fonts]
"""
import argparse
import csv
import json
import tempfile
from pathlib import Path
from urllib.request import urlopen
from xml.sax.saxutils import escape

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, PageBreak,
                               Table, TableStyle, Image, KeepTogether)
from reportlab.graphics.shapes import Drawing, Rect, String, Line, Polygon

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'analysis'
FIG = ROOT / 'figures'
BLUE = colors.HexColor('#183c5a')
TEAL = colors.HexColor('#087d82')
GRAY = colors.HexColor('#526271')
PALE = colors.HexColor('#eef4f8')
WIDTH = 180 * mm
TEXT_TRANSLATION = str.maketrans({'−': '-', '①': '(1)', '②': '(2)', '③': '(3)', '④': '(4)', '⑤': '(5)'})


def supported_text(value):
    # Nanum Gothic lacks U+2212 and circled digits; preserve their meaning visibly.
    return str(value).translate(TEXT_TRANSLATION)


def read_data():
    report = json.loads((DATA / 'speed_analysis.json').read_text())
    notes = json.loads((ROOT / 'measurement_notes.json').read_text())
    with (DATA / 'speed_samples.csv').open(encoding='utf-8-sig') as f:
        samples = list(csv.DictReader(f))
    return report, notes, samples


def write_calibration(report):
    trials = report['trials'][:5]
    k = np.array([r['summary']['approx_rx_divisor_counts_per_kmh'] for r in trials])
    fit = float((k @ k) / k.sum())
    rows = []
    for r in trials:
        s = r['summary']
        row = {'run': s['run'], 'command_raw': s['command_raw'],
               'measured_average_kmh': s['measured_average_kmh'],
               'approx_raw_mean': s['approx_window_raw_time_mean']}
        for divisor in (10., 11.65, 12.):
            label = str(divisor).replace('.', '_')
            estimate = s['approx_window_raw_time_mean'] / divisor
            row['average_kmh_div_' + label] = estimate
            row['error_kmh_div_' + label] = estimate - s['measured_average_kmh']
            row['relative_error_pct_div_' + label] = 100 * (estimate / s['measured_average_kmh'] - 1)
        rows.append(row)
    with (DATA / 'calibration_candidates.csv').open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n'); w.writeheader(); w.writerows(rows)
    sensitivity = []
    for delay in (0, .25, .5, 1.):
        ks = np.array([next(q['approx_rx_divisor_counts_per_kmh'] for q in r['arrival_timing_sensitivity']
                            if q['assumed_arrival_before_estop_s'] == delay) for r in trials])
        sensitivity.append({'assumed_arrival_before_estop_s': delay,
                            'fitted_common_divisor': float((ks @ ks) / ks.sum())})
    result = {'distance_least_squares_divisor': fit, 'rounded_candidate': 11.65,
              'scope': 'In-sample fit to runs 1-5 under the approximate EStop arrival assumption. Not independently validated.',
              'timing_sensitivity': sensitivity}
    (DATA / 'calibration_summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return result, rows


def make_figures(report, samples, calibration):
    FIG.mkdir(exist_ok=True)
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False,
                         'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    palette = ('#1766a6', '#dd6c20')
    fig, axes = plt.subplots(3, 1, figsize=(7.1, 8.6))
    for ax, command in zip(axes, (70, 100, 150)):
        for r in report['trials']:
            s = r['summary']
            if s['command_raw'] != command:
                continue
            a = [x for x in samples if int(x['run']) == s['run'] and x['automatic_trial_sample'] == 'True']
            x = [float(z['time_from_command_step_s']) for z in a]
            y = [float(z['speed_raw']) for z in a]
            color = palette[(s['run'] - 1) % 2]
            ax.plot(x, y, lw=1.5, color=color, label='시험 {}'.format(s['run']))
            ax.plot(x[-1], y[-1], 's', color=color, ms=4)
        ax.axhline(command, color='#444444', ls='--', lw=1, label='고정 명령 {}'.format(command))
        ax.set(title='입력 원시값 {}'.format(command), xlabel='명령 반영 후 시간 (s)',
               ylabel='수신 속도 원시값', ylim=(0, command * 1.14), xlim=(0, None))
        ax.grid(alpha=.2); ax.legend(loc='lower right', fontsize=8)
    fig.tight_layout(h_pad=1.4)
    fig.savefig(str(FIG / 'response_curves.png'), dpi=200); plt.close(fig)
    for group, run_ids in enumerate(((1, 2, 3), (4, 5, 6)), 1):
        fig, axes = plt.subplots(3, 1, figsize=(7.1, 8.4))
        for ax, run in zip(axes, run_ids):
            s = report['trials'][run - 1]['summary']
            a = [x for x in samples if int(x['run']) == run]
            t = np.array([float(z['bag_time_s']) for z in a])
            raw = np.array([float(z['speed_raw']) for z in a])
            feedback = np.array([float(z['feedback_speed_recorded']) * 10 for z in a])
            cmd = np.array([float(z['command_raw_last_received']) if z['command_raw_last_received'] else 0 for z in a])
            ymax = s['command_raw'] * 1.15
            ax.fill_between(t, 0, ymax, where=np.array([z['phase'] == 'manual' for z in a]),
                            step='post', color='#dddddd', label='수동')
            ax.fill_between(t, 0, ymax, where=np.array([z['phase'] == 'estop' for z in a]),
                            step='post', color='#f6d0d0', label='EStop')
            if s['approx_timed_window_usable']:
                ax.axvspan(s['approx_window_start_bag_s'], s['approx_window_end_bag_s'],
                           color='#d8ebfb', alpha=.65, label='측정 구간 근사')
            ax.step(t, cmd, where='post', color='#444444', ls='--', lw=1, label='명령')
            ax.plot(t, raw, color=palette[0], lw=1.4, label='raw')
            ax.step(t, feedback, where='post', color=palette[1], lw=.8, label='feedback × 10')
            ax.set(title='시험 {} / 입력 {} / {:.2f}초 / {:.2f}km/h'.format(
                run, s['command_raw'], s['measured_duration_s'], s['measured_average_kmh']),
                xlabel='bag 시작 후 시간 (s)', ylabel='원시 단위', xlim=(0, s['bag_duration_s']), ylim=(0, ymax))
            ax.grid(alpha=.2)
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='lower center', ncol=3, fontsize=8)
        fig.tight_layout(rect=(0, .055, 1, 1), h_pad=1.2)
        fig.savefig(str(FIG / ('timelines_{}.png'.format(group))), dpi=200); plt.close(fig)
    x = [z['assumed_arrival_before_estop_s'] for z in calibration['timing_sensitivity']]
    y = [z['fitted_common_divisor'] for z in calibration['timing_sensitivity']]
    fig, ax = plt.subplots(figsize=(7.1, 2.9))
    ax.plot(x, y, 'o-', color=palette[0]); ax.axhline(10, color='#777777', ls='--', label='현재 코드의 분모 10')
    for xx, yy in zip(x, y): ax.annotate('{:.2f}'.format(yy), (xx, yy), xytext=(5, 7), textcoords='offset points', fontsize=9)
    ax.set(xlabel='실제 도착이 EStop보다 앞섰다고 가정한 시간 (s)', ylabel='공통 보정 분모 k', ylim=(9.65, 12.1))
    ax.grid(alpha=.2); ax.legend(fontsize=8); fig.tight_layout()
    fig.savefig(str(FIG / 'timing_sensitivity.png'), dpi=200); plt.close(fig)


def register_fonts(font_dir):
    font_dir.mkdir(parents=True, exist_ok=True)
    for name in ('NanumGothic-Regular.ttf', 'NanumGothic-Bold.ttf', 'OFL.txt'):
        p = font_dir / name
        if not p.exists():
            with urlopen('https://raw.githubusercontent.com/google/fonts/main/ofl/nanumgothic/' + name, timeout=30) as r:
                p.write_bytes(r.read())
    pdfmetrics.registerFont(TTFont('Korean', str(font_dir / 'NanumGothic-Regular.ttf')))
    pdfmetrics.registerFont(TTFont('KoreanBold', str(font_dir / 'NanumGothic-Bold.ttf')))
    pdfmetrics.registerFontFamily('Korean', normal='Korean', bold='KoreanBold', italic='Korean', boldItalic='KoreanBold')


def styles():
    return {
        'body': ParagraphStyle('body', fontName='Korean', fontSize=9.3, leading=14.2, spaceAfter=7,
                               wordWrap='CJK', textColor=colors.HexColor('#243444')),
        'small': ParagraphStyle('small', fontName='Korean', fontSize=8, leading=12, spaceAfter=5,
                                wordWrap='CJK', textColor=GRAY),
        'title': ParagraphStyle('title', fontName='KoreanBold', fontSize=26, leading=36, spaceAfter=16, textColor=BLUE),
        'h1': ParagraphStyle('h1', fontName='KoreanBold', fontSize=17, leading=24, spaceAfter=14, textColor=BLUE, keepWithNext=True),
        'h2': ParagraphStyle('h2', fontName='KoreanBold', fontSize=11.5, leading=17, spaceBefore=9, spaceAfter=6, textColor=TEAL, keepWithNext=True),
        'cell': ParagraphStyle('cell', fontName='Korean', fontSize=8.4, leading=12.3, wordWrap='CJK'),
        'head': ParagraphStyle('head', fontName='KoreanBold', fontSize=8.5, leading=12.5, wordWrap='CJK', textColor=colors.white),
        'code': ParagraphStyle('code', fontName='Korean', fontSize=8.2, leading=13, wordWrap='CJK',
                               backColor=PALE, borderPadding=8, spaceBefore=5, spaceAfter=10),
    }


class Report:
    def __init__(self): self.story = []; self.s = styles()
    def p(self, text, style='body'): self.story.append(Paragraph(supported_text(text), self.s[style]))
    def title(self, text): self.p(text, 'h1')
    def sub(self, text): self.p(text, 'h2')
    def page(self): self.story.append(PageBreak())
    def gap(self, size=4): self.story.append(Spacer(1, size * mm))
    def table(self, headers, rows, widths):
        data = [[Paragraph(escape(supported_text(t)), self.s['head']) for t in headers]]
        data.extend([[Paragraph(escape(supported_text(t)), self.s['cell']) for t in row] for row in rows])
        table = Table(data, colWidths=[v * mm for v in widths], repeatRows=1, hAlign='LEFT')
        table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), BLUE), ('VALIGN', (0, 0), (-1, -1), 'TOP'),
            ('LEFTPADDING', (0, 0), (-1, -1), 7), ('RIGHTPADDING', (0, 0), (-1, -1), 7),
            ('TOPPADDING', (0, 0), (-1, -1), 6), ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, PALE]),
            ('LINEBELOW', (0, 0), (-1, 0), .7, BLUE),
            ('LINEBELOW', (0, 1), (-1, -1), .25, colors.HexColor('#dce5eb')),
        ]))
        self.story.extend([table, Spacer(1, 3 * mm)])
    def figure(self, path, width_mm, height_mm):
        self.story.append(Image(str(path), width=width_mm * mm, height=height_mm * mm, kind='proportional'))
    def code(self, lines): self.p('<br/>'.join(escape(line).replace(' ', '&#160;') for line in lines), 'code')


def flow_diagram():
    d = Drawing(WIDTH, 92)
    labels = [('상위 PC / ROS', 'drive · mode'), ('시리얼 드라이버', 'TX 14 bytes / RX 18 bytes'), ('ERP 내부 제어기', '내부 센서 구성은 미확인')]
    for i, (a, b) in enumerate(labels):
        x = i * 176
        d.add(Rect(x, 20, 157, 62, rx=5, ry=5, fillColor=PALE, strokeColor=BLUE, strokeWidth=.7))
        d.add(String(x + 78, 58, a, fontName='KoreanBold', fontSize=10, textAnchor='middle', fillColor=BLUE))
        d.add(String(x + 78, 37, b, fontName='Korean', fontSize=7.5, textAnchor='middle', fillColor=GRAY))
        if i < 2:
            d.add(Line(x + 158, 52, x + 173, 52, strokeColor=TEAL, strokeWidth=1.4))
            d.add(Polygon([x + 173, 52, x + 168, 55, x + 168, 49], fillColor=TEAL, strokeColor=TEAL))
    return d


def pages_one_to_five(r, report, notes):
    r.p('ERP42 DIAGNOSTICS / FIELD RECORD', 'small')
    r.gap(8)
    r.p('ERP42 조향·속도<br/>진단 및 측정 보고서', 'title')
    r.p('원시 시리얼 데이터부터 저속 보정 후보까지', 'h2')
    r.p('측정일 2026-11-21 · 정리일 2026-11-22 · 시간대 Asia/Seoul', 'small')
    r.p('대화에서 제공된 관측값, 현재 저장소 코드, 여섯 개 rosbag을 근거로 작성했습니다. 차량 내부 ECU/펌웨어를 조사한 문서가 아니며, 확정된 사실과 임시 추정을 구분합니다.')
    r.gap()
    r.table(['구분', '현재 정리된 결과'], [
        ['확인', '입력 70·100·150은 자동 주행 중 일정했다. 수신 속도 원시값은 주행 중 계속 상승하는 경향을 보였다.'],
        ['확인', '원시값 2,539개가 수신 바이트 해석과 일치했다. 일반 피드백은 raw / 10의 정수 나눗셈 결과와 일치했다.'],
        ['확인', '14.2m 정지 출발 시험의 평균 속도는 입력 / 18.71에 가깝다. 정속 주행의 환산식은 아니다.'],
        ['후보', '수신 속도 보정 분모 약 11.65. EStop을 도착으로 근사한 다섯 기록에 맞춘 값으로, 코드에는 미적용이다.'],
        ['목표', '실사용 1~3km/h, 허용 오차 약 ±0.3km/h. 해당 범위의 독립적인 실측 검증이 필요하다.'],
        ['미확정', '40초 출발 지연 원인, 조향 중앙 오프셋, 차량별 물리 단위와 내부 센서 배선, 정속 명령 보정 계수.'],
    ], [24, 156])
    r.p('코드 변경: feedback_raw / feedback_bytes 추가, 음수 조향 경계 처리 수정. 속도 /10, 조향 /71 및 구동 명령 스케일은 현재 코드 값을 유지합니다.', 'small')
    r.p('문서 구성: 데이터 경로·코드(2~3쪽), 조향(4쪽), 시험 조건(5쪽), 그래프(6~8쪽), 보정 분석(9~10쪽), 후속 시험(11쪽), INS(12쪽), 재현·근거(13~14쪽).', 'small')
    r.page()

    r.title('02 / 시리얼 데이터와 ROS 토픽')
    r.story.append(flow_diagram()); r.gap()
    r.p('/erp42_serial/feedback은 상위 PC의 ROS 드라이버가 발행하는 토픽입니다. 그 안의 속도·조향·모드·엔코더 값은 ERP가 시리얼로 보낸 수신 프레임을 해석한 것입니다. /command는 PC 측 명령 기록이며 차량의 수신 확인 응답을 뜻하지 않습니다.')
    r.table(['토픽', '내용'], [
        ['/erp42_serial/drive', 'DriveCmd: KPH와 Deg를 현재 드라이버는 원시 명령값으로 사용'],
        ['/erp42_serial/mode', 'ModeCmd: MorA / EStop / Gear 등 송신 상태 설정'],
        ['/erp42_serial/feedback', 'SerialFeedBack: raw speed / 10, raw steer / 71. 현재는 소수점 절삭'],
        ['/erp42_serial/feedback_raw', '같은 메시지 형식. speed·steer만 환산 전 원시값으로 교체'],
        ['/erp42_serial/feedback_bytes', 'UInt8MultiArray: STX/CRLF를 포함한 수신 프레임 18바이트'],
    ], [70, 110])
    r.sub('현재 드라이버가 해석하는 수신 프레임: 0부터 시작하는 인덱스')
    r.table(['바이트', '해석'], [
        ['0~2', 'STX = 0x53 0x54 0x58'], ['3 / 4 / 5', 'MorA / EStop / Gear'],
        ['6~7', '속도 uint16: byte[6] + byte[7] × 256'],
        ['8~9', '조향 int16: 먼저 byte[8] + byte[9] × 256, 32768 이상이면 65536 차감'],
        ['10', '브레이크 피드백'], ['11~14', '엔코더 int32, little-endian'],
        ['15 / 16~17', 'alive / CR LF (0x0D 0x0A)'],
    ], [26, 154])
    r.p('한 바이트는 0~255를 표현합니다. 두 바이트는 16비트이며 16진수 네 자리로 표시할 수 있습니다. 상위 바이트에 256을 곱하는 이유는 8비트 자릿값 때문입니다. 예: [0xE8, 0x03] → 1000. 조향 [0x4C, 0xF8] → 63564 − 65536 = −1972.', 'small')
    r.p('수신 RX의 속도·조향은 little-endian이지만, 현재 송신 TX 14바이트에서는 상위 바이트를 먼저 배치합니다. 코드의 /10·/71만으로 이 차량의 실제 물리 단위가 검증된 것은 아닙니다.', 'small')
    r.page()

    r.title('03 / 코드 점검과 이번 변경')
    r.table(['항목', '소스에서 확인한 동작'], [
        ['명령 반영', 'DriveCallback은 KPH·Deg를 그대로 송신 구조체에 저장한다. 이 경로에서 속도를 점진적으로 올리는 램프 계산은 없다.'],
        ['피드백 환산', 'SPEED_FACTOR=10, STEER_FACTOR=71. speed_now와 steer_now가 정수이므로 현재 나눗셈에서 소수점이 사라진다.'],
        ['AUTO 중앙 정렬', 'ModeCallback은 모드·EStop·기어만 저장한다. AUTO 진입 때 조향 0을 넣는 코드는 없다. 노드 생성 시 InitData에서는 속도·조향을 0으로 초기화한다.'],
        ['반복 주기', 'Run은 spinOnce → Read → Write 순서. 설정 루프는 50Hz지만 이번 기록의 수신·command는 약 19.6Hz였다.'],
        ['속도 상한', '상위 Control.cpp에는 목표 속도 0~24 제한과 ×10 송신이 있다. /drive 직접 발행은 그 상위 제한을 우회한다. 24는 차량의 검증된 허용 최고속도를 뜻하지 않는다.'],
        ['기존 제약', '송신 brake 바이트는 0으로 고정되어 있고 명령 유효시간 감시는 없다. /command 발행만으로 시리얼 쓰기 성공·차량 수신을 보증하지 않는다.'],
    ], [30, 150])
    r.sub('실제로 반영한 변경')
    r.p('① feedback_raw 발행자와 원시 속도·조향 복사 추가. ② feedback_bytes 발행자로 수신 프레임 전체 보존. ③ 조향 부호 처리 조건을 “&gt; 32768”에서 “&gt;= 32768”로 수정하여 0x8000을 −32768로 해석.')
    r.p('두 진단 토픽의 메시지 발행은 구독자가 있을 때 수행하며 AUTO 조건으로 제한하지 않습니다. 이번 bag의 수동 구간에도 원시값이 기록되었습니다. 실측 정상 조향 범위보다 훨씬 큰 −32768 경계 수정은 디코딩의 정확성 수정입니다.')
    r.sub('이전 원시 토픽 미출력과 실행 경로')
    r.p('대화 중 구형 실행 파일/다른 워크스페이스가 선택되는 문제가 확인되었습니다. 셸에서 setup.bash를 source하는 것만으로 이미 실행 중인 노드는 교체되지 않습니다. 실행 전에 rospack 경로와 빌드된 바이너리, 실행 중인 노드의 경로를 함께 확인해야 합니다.')
    r.code(['source /opt/ros/noetic/setup.bash', 'source ~/ERP_2024_github/devel/setup.bash', 'rospack find erp42_serial', 'rostopic info /erp42_serial/feedback_raw'])
    r.p('launch 터미널의 “Sending Packet”, 조향 변수, 16진 바이트 출력은 writeUpdate의 송신 측 로그입니다. 실제 차량 수신이나 구동 완료 확인으로 해석하지 않습니다.', 'small')
    r.p('검증: erp42_serial_node 빌드 통과. 기존 bag 2,539개 원시 메시지의 바이트·환산 결과 대조 통과. 보고서 작성 과정에서 실제 차량 구동 시험은 수행하지 않았습니다.', 'small')
    r.page()

    r.title('04 / 조향 관측 기록과 해석')
    r.sub('초기 보고된 원시값과 각도')
    r.table(['방향', '원시 조향값', '/71 결과', '당시 보고한 각도'], [
        ['우측 최대', '+1947', '+27.42°', '약 32°'],
        ['좌측 최대', '−1972', '−27.77°', '약 22°'],
    ], [33, 38, 38, 71])
    r.p('각도 크기만 비교하면 우측 약 −4.58°, 좌측 약 +5.77° 차이입니다. 어느 바퀴를 같은 기준으로 측정했는지 확정되지 않았고, 이후 관측과 동시 측정한 원시값도 아니므로 좌우 환산 계수를 바로 따로 정하지 않았습니다.')
    r.sub('이후 정지 상태에서 측정한 양쪽 앞바퀴')
    r.table(['상태', '왼쪽 앞바퀴', '오른쪽 앞바퀴'], [
        ['명목상 직진', '0°', '오른쪽으로 약 5°'],
        ['최대 좌회전', '좌측 약 27.5°', '좌측 약 19°'],
        ['최대 우회전', '우측 약 20°', '우측 약 28°'],
        ['양 끝 사이 총 범위', '약 47.5°', '약 47°'],
    ], [54, 63, 63])
    r.p('최대 회전에서 안쪽 바퀴의 각도가 더 큰 양상은 조향 기하와 양립합니다. 그러나 직진 위치에서 왼쪽 0°·오른쪽 +5°인 상태는 공통 조향 오프셋만으로 양쪽을 동시에 0°로 만들 수 있는지 별도 확인이 필요합니다. 기준선·토·링크 정렬과 측정 반복성을 구분해야 합니다.')
    r.sub('4° 오른쪽으로 향했다는 관측')
    r.p('차량이 약 2.8m 이동한 후 진행 방향이 오른쪽으로 약 4° 바뀌었다는 육안 관측입니다. 바닥은 대리석이었고 정밀한 헤딩 측정은 아니었습니다. 이것은 바퀴 조향각 4°를 뜻하지 않으므로 조향 명령에 −4°를 바로 더하는 근거로 사용하지 않았습니다.')
    r.sub('현재 결정과 다음 확인')
    r.p('조향 보정 오프셋은 미적용입니다. 동일한 바퀴·동일한 기준선에서 직진, 좌·우 조향을 반복하고, 각 측정과 동시에 명령 Deg·feedback_raw.steer·AUTO/MANUAL 상태를 기록해야 합니다. AUTO 전환 때의 중앙 정렬이 보인다면 초기화된 명령 0과 차량 내부 동작을 구분합니다.')
    r.p('차량 내부 ECU, 조향 센서, 모터 컨트롤러 사이의 구체적인 배선·버스 구성은 이 저장소와 bag으로 확인되지 않습니다. 모델명·제어기 라벨·펌웨어 정보 또는 해당 차량의 제조사 자료가 추가 근거입니다.', 'small')
    r.page()

    r.title('05 / 속도 시험 조건과 원본 기록')
    r.p('이번 여섯 시험은 모두 정지 출발부터 14.2m까지 측정한 시간입니다. 사용자가 이후 명확히 확인했으며, 별도의 1.42m 가속 구간은 이번 시험에 적용하지 않습니다. 정확한 시작·도착 마커나 영상은 없습니다.')
    rows = []
    for trial in report['trials']:
        s = trial['summary']
        rows.append([s['run'], s['bag'][-12:-4], s['command_raw'], '{:.2f}'.format(s['measured_duration_s']),
                     '{:.3f}'.format(s['measured_average_kmh']), s['raw_sample_count']])
    r.table(['시험', '파일 시각', '입력 raw', '시간(s)', '평균(km/h)', 'raw 개수'], rows, [15, 35, 30, 30, 37, 33])
    r.p('파일 공통 접두사: erp42_speed_check_2026-11-21- / 확장자: .bag. 파일명 시각과 첫 메시지 시각은 약간 다를 수 있습니다. 원본 파일의 SHA-256은 analysis/speed_analysis.json에 기록했습니다.', 'small')
    r.sub('분석 구간을 정한 방법')
    r.p('자동 주행은 MorA=1, EStop=0, Gear=0이며 해당 입력 명령이 유지되는 구간으로 구분했습니다. EStop 및 수동 조작 구간은 자동 주행 지표에서 제외했습니다. 시험 1·6에는 뒤쪽 수동 주행이 있습니다.')
    r.p('시험 1~5: 사용자는 도착 직후 EStop을 눌렀다고 설명했습니다. 따라서 첫 EStop=1 시각을 도착의 근사값으로 잡고, 측정 시간만큼 거슬러 올라가 수신값을 적분했습니다. 시험 6: 도착 후 추가 이동이 있어 이 방식의 환산 계수 계산에서는 제외했습니다.')
    r.sub('기록 품질')
    r.p('feedback_raw / feedback_bytes 2,539개가 모두 대응했습니다. 바이트 해석과 원시값의 불일치는 0개였고, 대응하는 일반 feedback의 속도·조향 환산 불일치도 0개였습니다. 일반 feedback만 추가로 잡힌 메시지는 시험 2·6에서 각각 1개입니다. raw 수신률은 약 19.608Hz, 최대 메시지 간격은 약 0.052초였습니다.')
    r.sub('이전 시험은 조건이 달라 별도 보관')
    r.table(['입력', '이전 시간', '이전 조건'], [
        ['70', '13.40 / 13.92초', '1.42m 가속 구간 후 별도의 14.2m 측정'],
        ['100', '9.63 / 9.63초', '동일. 이번 여섯 시험의 회귀 계산에 합치지 않음'],
    ], [25, 45, 110])
    r.p('초기 약 40초 출발 지연은 raw 10, 조향 ±1065를 20Hz로 번갈아 주던 시험에서 보고된 현상입니다. 이번 raw 70·100·150 bag만으로 그 저입력 조건의 원인을 확정하지 않습니다.', 'small')
    r.page()


def pages_six_to_fourteen(r, report, notes, calibration, candidates):
    r.title('06 / 일정한 명령과 상승하는 수신 속도')
    r.p('각 입력의 두 곡선을 명령이 /command에 나타난 시점으로 맞췄습니다. 점선은 고정 입력이며, 끝점은 EStop 직전입니다. 시험 6은 도착 후 추가 이동을 포함합니다.', 'small')
    r.figure(FIG / 'response_curves.png', 174, 211)
    r.p('같은 입력에서 응답 형태가 상당히 비슷합니다. 수신값의 상승은 관측 사실이지만, 실제 가속과 차량 내부 속도 추정·필터 지연을 이 자료만으로 분리할 수는 없습니다.', 'small')
    r.page()

    r.title('07 / 전체 기록과 제외 구간: 시험 1~3')
    r.p('파란 선은 원시 속도, 주황 선은 일반 feedback × 10입니다. 파란 배경은 도착을 EStop으로 근사한 시간 구간, 붉은 배경은 EStop, 회색은 수동 구간입니다.', 'small')
    r.figure(FIG / 'timelines_1.png', 174, 206)
    r.p('시험 1의 후반 수동 이동과 모든 EStop 구간은 자동 주행 지표에서 제외했습니다. feedback의 계단은 원시값의 해상도와 정수 나눗셈이 함께 만든 결과입니다.', 'small')
    r.page()

    r.title('08 / 전체 기록과 제외 구간: 시험 4~6')
    r.p('시험 6은 도착 후 추가로 이동했다는 사용자 설명에 따라 수신 환산 계수 계산에서 제외했습니다. 입력 150의 응답 형태와 사용자 실측 평균 속도 분석에는 포함했습니다.', 'small')
    r.figure(FIG / 'timelines_2.png', 174, 206)
    r.p('명령이 높게 유지되어도 EStop 이후 차량 피드백은 정지로 바뀔 수 있습니다. 그래서 모드만 AUTO인지 보는 대신 EStop·기어·시간 구간을 함께 사용했습니다.', 'small')
    r.page()

    r.title('09 / 평균 속도와 수신 환산의 구분')
    fit = report['command_to_measured_average_fit']
    r.sub('A. 입력 명령과 14.2m 구간 평균 속도')
    r.code(['실측 평균 속도[km/h] = 14.2 / 측정시간[s] × 3.6',
            '이번 여섯 시험: 평균 속도 ≈ 입력 원시값 / {:.2f}'.format(fit['through_origin_divisor']),
            '원점 통과 회귀 R² = {:.4f}'.format(fit['through_origin_centered_r_squared'])])
    r.p('이는 정지 출발과 가속이 포함된 짧은 구간의 평균입니다. 차량이 충분히 달린 뒤의 정속 환산식이나, feedback의 단위 변환 분모로 사용하지 않습니다. 앞서 제시했던 약 18.8도 이 제한을 적용해야 합니다.')
    r.sub('B. 수신 원시값과 실측 거리')
    r.code(['실제 속도[km/h] ≈ raw(t) / k 라고 가정',
            '예측 거리[m] = ∫ raw(t) dt / (3.6 × k)',
            '구간별 분모 k = ∫ raw(t) dt / (3.6 × 14.2)'])
    rows = []
    for trial in report['trials'][:5]:
        s = trial['summary']
        rows.append([s['run'], s['command_raw'], '{:.2f}'.format(s['approx_window_raw_time_mean']),
                     '{:.2f}'.format(s['approx_window_distance_if_divisor_10_m']),
                     '{:.2f}'.format(s['approx_rx_divisor_counts_per_kmh'])])
    r.table(['시험', '입력', 'raw 시간 평균', '/10 거리(m)', '개별 분모 k'], rows, [18, 25, 46, 46, 45])
    r.p('EStop 시점을 도착으로 근사하면 /10은 14.2m보다 약 8~20% 긴 거리를 계산합니다. 이 차이에는 실제 환산 오차와 수동 시간 측정·정지 반응의 오차가 함께 포함될 수 있습니다.')
    r.sub('이번 자료에서 확인한 지연과 해상도')
    r.table(['관측', '값과 의미'], [
        ['drive → command', '최초 양의 입력 반영 약 0.051~0.093초. 자동 구간에서 입력은 고정값 유지.'],
        ['첫 양의 raw', 'command 이후 약 0.82~1.17초. 실제 차체 출발 시점을 직접 측정한 값은 아님.'],
        ['원시값 단계', '0, 6, 12, 18, 24, 31, 37, …, 142, 149. 원시값부터 약 6~7 간격.'],
        ['일반 feedback', '예: raw 68 → speed 6. 6.8이 보존되지 않는 정수 나눗셈.'],
    ], [42, 138])
    r.page()

    r.title('10 / 공통 보정 후보와 시간 오차')
    r.p('첫 다섯 시험의 거리 오차 제곱합을 작게 만드는 공통 분모는 <b>{:.5f}</b>입니다. 실용적인 임시 후보를 <b>11.65</b>로 정리합니다. 동일 자료에 맞춘 결과이며 독립적인 검증 결과는 아닙니다.'.format(calibration['distance_least_squares_divisor']))
    r.table(['시험 / 입력', '실측 평균', 'raw / 11.65 평균', '차이(km/h)'], [
        ['{} / {}'.format(x['run'], x['command_raw']), '{:.2f}'.format(x['measured_average_kmh']),
         '{:.2f}'.format(x['average_kmh_div_11_65']), '{:+.2f}'.format(x['error_kmh_div_11_65'])] for x in candidates
    ], [40, 40, 55, 45])
    r.p('분모 11.65일 때 기존 구간 평균의 상대 오차는 약 −7.5~+3.2%입니다. 분모 12도 후보가 될 수 있지만 같은 다섯 시험에서 약 −10.1~+0.2%의 오차가 남습니다. 어느 값도 현재 구동 코드에 적용하지 않았습니다.')
    r.sub('도착과 EStop의 간격을 달리 가정하면 계수도 변함')
    r.figure(FIG / 'timing_sensitivity.png', 177, 72)
    r.p('위 그래프는 실제 반응 시간을 측정한 결과가 아니라 민감도 분석입니다. 다섯 시험의 도착이 모두 EStop보다 0.5초 빨랐다고 가정하면 공통 분모는 약 10.83이 됩니다. 그러므로 11.65를 차량 고유 단위라고 확정할 수는 없습니다.', 'small')
    r.sub('1~3km/h, ±0.3km/h 목표에 대한 판단')
    r.p('약 7.5% 수준의 비율 오차가 저속에서도 유지된다면 3km/h에서 약 0.23km/h입니다. 다만 보정에 사용한 실측 평균은 3.56~7.93km/h이므로 1~3km/h에 대한 외삽 검증이 필요합니다. 이 수치로 저속 정확도를 보장하지 않습니다.')
    r.p('현재 정수 나눗셈을 그대로 두면 소수점 정보가 사라집니다. 실제 보정 구현 시에는 수신 환산용 실수 계수를 별도로 사용해야 합니다. 원시값의 약 6~7 단계를 /11.65로 환산해도 약 0.5~0.6km/h 간격이므로 순간 표시값과 구간 평균의 정확도를 구분해야 합니다.', 'small')
    r.page()

    r.title('11 / 계수 하나를 검증하는 추가 기록')
    r.p('권장 방식은 <b>정지 위치 → 전진 주행 → 완전 정지 위치</b> 사이의 실제 거리를 측정하고 전체 이동을 기록하는 것입니다. EStop 시점을 도착선 통과 시점으로 대신 쓰는 오차를 줄일 수 있습니다.')
    r.table(['순서', '측정 방법'], [
        ['1', '정지 상태에서 bag을 먼저 기록한다. 앞축 중심 등 같은 차량 기준점의 출발 위치를 표시한다.'],
        ['2', '전진 직선 주행 후 완전히 정지한다. 목표 저속 범위의 여러 조건에서 반복한다.'],
        ['3', '완전 정지 후에도 수초 동안 기록한 뒤 bag을 종료한다. 수동 복귀는 기록 종료 후 수행한다.'],
        ['4', '같은 차량 기준점으로 출발 위치부터 최종 정지 위치까지 실제 거리를 잰다. 제동 중 이동 거리도 포함한다.'],
        ['5', '전체 이동의 raw 적분으로 분모를 구한다. 일부 기록으로 계수를 정하고, 나머지 기록으로 검증한다.'],
    ], [18, 162])
    r.p('이 방식은 가속·감속이 포함돼도 환산 계수를 계산할 수 있습니다. 일정 속도 명령의 보정이 목적이라면 속도가 안정된 구간을 따로 잡아 입력 명령과 실측 속도를 비교해야 합니다.')
    r.sub('기록 명령 예시: 차량 명령을 발행하지 않는 수집 명령')
    r.code(['rosbag record -O erp42_low_speed_check.bag \\',
            '  /erp42_serial/drive /erp42_serial/mode \\',
            '  /erp42_serial/command /erp42_serial/feedback \\',
            '  /erp42_serial/feedback_raw /erp42_serial/feedback_bytes'])
    r.p('유효한 INS가 실제로 발행되고 있다면 해당 토픽도 함께 기록할 수 있습니다. 위치·속도 기준이 독립적인지, 유효 상태와 시간 동기화가 맞는지 함께 확인합니다.', 'small')
    r.sub('각 주행에 남길 기록')
    r.table(['항목', '기록할 값'], [
        ['주행 식별', 'bag 파일명, 날짜·시간, 입력값, 반복 번호'],
        ['거리 기준', '출발·최종 정지 위치 사이의 실측 거리, 차량 기준점, 거리 측정 방법'],
        ['조건', '전진/후진, 직선 여부, 바닥·경사, 적재·배터리 상태, 조향 명령'],
        ['검증 목적', '계수 추정용 / 독립 검증용, 구간 평균 오차 / 순간값 오차'],
    ], [36, 144])
    r.p('정지-정지의 전체 기록은 거리 스케일을 검증합니다. 순간 속도 오차 ±0.3km/h까지 판정하려면 정속 구간의 거리·시간 또는 유효한 외부 속도 기준이 추가로 필요합니다.', 'small')
    r.page()

    r.title('12 / IMU·INS와 차량 내부 정보의 범위')
    r.p('가속도만을 적분한 IMU 속도에는 바이어스와 자세 오차가 누적될 수 있습니다. 이 저장소에는 VectorNav INS 속도를 읽는 코드가 있으므로, 실제 센서가 이를 제공하고 유효한 상태라면 별도 속도 기준 후보가 됩니다. 이번 여섯 bag에는 INS 데이터가 없습니다.')
    r.table(['코드 / 토픽', '확인한 내용'], [
        ['vectornav / CMakeLists.txt', '실행 파일 vnpub는 src/main.cpp로 빌드된다. IDE에서 연 subtest2.cpp가 현재 vnpub의 빌드 입력인 것은 아니다.'],
        ['vectornav.launch', 'vnpub와 nmea_navsat_driver를 실행하고 vn200.yaml 설정을 읽는다. 설정 파일명만으로 실제 장착 모델을 확정할 수 없다.'],
        ['/vectornav/INS', 'Ins.msg의 nedVelX/Y/Z는 NED 방향 속도이며 메시지 정의상 단위는 m/s다.'],
        ['상태 정보', 'insStatus, velUncertainty 및 메시지 갱신 시각을 함께 확인한다. 실제 센서 모델에 따라 INS가 제공되지 않을 수 있다.'],
        ['모델·펌웨어', 'main.cpp에서 readModelNumber와 readFirmwareVersion을 호출해 출력한다. ERP 내부 제어기 정보와는 별개다.'],
    ], [58, 122])
    r.code(['수평 속도[km/h] = 3.6 × sqrt(nedVelX² + nedVelY²)',
            'rostopic list', 'rostopic type /vectornav/INS',
            'rostopic hz /vectornav/INS', 'rostopic echo -n 1 /vectornav/INS'])
    r.p('Ins.msg 주석상 mode 비트는 0~1, GPS fix 비트는 2, error 필드는 3~6이다. 상태를 단순히 0이 아닌 값이라고만 검사하지 않고, 모드·GPS fix·오류와 속도 불확실도를 함께 해석해야 한다. 실제 사용 장비의 상태 정의와 정확도 확인이 선행되어야 한다.', 'small')
    r.sub('현재 접근 가능한 것과 확인되지 않은 것')
    r.p('PC의 소스 코드와 bag으로는 외부 시리얼 프로토콜 및 PC에 연결된 센서 드라이버 경로를 확인할 수 있습니다. ERP 내부의 ECU 종류, 센서 연결 버스, 펌웨어 제어 로직이나 서비스 포트는 여기서 확인되지 않습니다.')
    r.p('사용자는 일반 ERP42로 알고 있으나 모델·연식·제어기 문서가 없는 상태입니다. 차량과 제어기 라벨, 납품 자료, 제조사의 해당 모델 프로토콜을 확인해야 합니다. 다른 ERP42 파생 모델의 단위나 내부 구성을 이 차량에 그대로 적용하지 않았습니다.')
    r.sub('40초 출발 지연의 현재 상태')
    r.p('원래 현상은 매 재출발에서 약 40초 지연된다는 사용자 보고입니다. 사용자는 이 이슈를 잠시 보류하고 조향·속도 환산을 먼저 확인하기로 했습니다. 현재 자료는 raw 10 조건의 지연 원인을 확정하지 않으며, 내부 제어·저속 응답과 PC 실행 경로를 구분하는 추가 기록이 필요합니다.')
    r.page()

    r.title('13 / 변경 이력, 검증과 재현 방법')
    sha = notes['driver_commit']
    url = notes['repository_url'] + '/commit/' + sha
    r.p('코드 변경 커밋: <link href="{}" color="#087d82">{}</link>'.format(url, sha), 'small')
    r.table(['항목', '결과'], [
        ['시리얼 노드 빌드', 'cmake --build build --target erp42_serial_node -- -j2 통과'],
        ['원본 bag 재해석', '2,539개 원시값과 바이트 프레임의 속도·조향·메타데이터 대응'],
        ['환산 검사', 'speed=trunc(raw/10), steer=trunc(raw/71)와 대응하는 모든 feedback 일치'],
        ['적용된 기능', '원시값·바이트 토픽과 음수 조향 경계 처리'],
        ['아직 적용하지 않은 후보', '속도 분모 11.65, 실수 환산, 조향 중앙 오프셋, 목표 속도 명령 보정'],
    ], [42, 138])
    r.sub('분석과 PDF 재생성')
    r.code(['source /opt/ros/noetic/setup.bash',
            'cd docs/erp42_diagnostics_20261122',
            '/usr/bin/python3 analysis/analyze_bags.py --bag-dir /path/to/bags',
            '/usr/bin/python3 build_report.py'])
    r.p('분석은 rosbag·NumPy·Matplotlib을, PDF는 ReportLab을 사용합니다. ROS master 연결이나 차량 명령 발행은 하지 않습니다. PDF용 Nanum Gothic 글꼴은 Google Fonts 저장소의 SIL OFL 배포본을 임시 폴더에 캐시하며 글꼴을 PDF에 포함합니다.', 'small')
    r.sub('동봉 파일')
    r.table(['파일 / 폴더', '내용'], [
        ['ERP42_diagnostics_report_2026-11-22.pdf', '본 문서'],
        ['measurement_notes.json', '사용자 관측, 조건 정정, 목표 오차, 코드 커밋과 소스 해시'],
        ['analysis/speed_summary.csv', '시험별 실측·추정 구간·응답 지표'],
        ['analysis/speed_samples.csv', '2,539개 원시값·피드백·바이트·명령 정렬 기록'],
        ['analysis/speed_analysis.json', '가정, 검증 개수, 파일 SHA-256, 회귀 및 상세 결과'],
        ['analysis/calibration_*.csv / *.json', '공통 후보와 시간 가정에 대한 민감도'],
        ['analysis/analyze_bags.py / build_report.py', '재현 가능한 분석·보고서 생성 코드'],
        ['figures/ 및 analysis/*.png', '보고서용 그래프와 원래 분석 그래프'],
    ], [82, 98])
    r.p('원본 bag은 저장소의 기존 *.bag 제외 규칙에 따라 Git 추적에 추가하지 않습니다. 사용자에게 제공하는 별도 ZIP에는 원본 여섯 bag과 본 보고서·분석 산출물을 함께 포함합니다.', 'small')
    r.page()

    r.title('14 / 근거 파일과 남은 확인 사항')
    r.p('아래 코드는 기록된 시리얼 변경 커밋 기준으로 참조합니다. 측정값은 사용자가 대화에서 제공한 관측이고, 정밀 계측으로 다시 확인한 값으로 표시하지 않았습니다.', 'small')
    sources = [
        ('src/erp42_serial/include/erp42_serial.h', '36~41', '환산 상수, 프레임 크기'),
        ('src/erp42_serial/src/erp42_serial.cpp', '36~44, 105~153', '토픽 및 원시 수신 해석'),
        ('src/erp42_serial/src/erp42_serial.cpp', '56~86, 176~197', '송신 바이트·모드·명령 처리'),
        ('src/erp42_serial/src/SerialPort/serial_port.cpp', 'Configure / Read / Write', '시리얼 포트 설정과 입출력'),
        ('src/ERP2024/src/Control/Control.cpp', '348~420', '상위 속도 제한, ×10, 조향 74.074 사용'),
        ('src/vectornav/vectornav/src/main.cpp', '274~275, 680~740', '장비 식별과 INS 메시지 채움'),
        ('src/vectornav/vectornav/msg/Ins.msg', '메시지 정의', 'NED 속도 단위, 상태 비트'),
        ('src/vectornav/vectornav/CMakeLists.txt', '48', 'vnpub의 실제 빌드 소스'),
        ('src/vectornav/vectornav/launch/vectornav.launch', 'launch 정의', '센서 실행·파라미터 로드'),
    ]
    r.table(['저장소 경로', '위치', '근거'], sources, [101, 32, 47])
    r.p('참고: 상위 제어 코드의 조향 환산 74.074와 시리얼 피드백 /71은 서로 다릅니다. 이번 직접 /drive 시험과 상위 제어 경로를 섞어 보정하지 않아야 합니다.', 'small')
    r.sub('원본 파일 식별: SHA-256 앞 16자리')
    r.table(['파일명 끝부분', 'SHA-256 접두사'], [
        [t['summary']['bag'][-12:], t['bag_sha256'][:16]] for t in report['trials']
    ], [75, 105])
    r.p('전체 해시는 analysis/speed_analysis.json에 있습니다. 마지막 bag의 늦은 정지와 수동 조작은 삭제하지 않고 원본 그대로 보존합니다.', 'small')
    r.sub('다음 판단에 필요한 정보')
    r.p('① 실제 1~3km/h 조건의 별도 검증 기록. ② 같은 기준점으로 측정한 출발-완전 정지 거리. ③ 양쪽 바퀴 각도와 raw의 동시 측정. ④ 실제 차량 모델·펌웨어 자료. ⑤ 필요 시 유효 INS 속도와 시간 동기화. 이러한 정보가 확보되면 보정 후보의 적용 범위를 좁히고 하나의 계수로 충분한지 판단할 수 있습니다.')


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor('#dbe5ed'))
    canvas.line(15 * mm, 13 * mm, 195 * mm, 13 * mm)
    canvas.setFont('Korean', 7)
    canvas.setFillColor(GRAY)
    canvas.drawString(15 * mm, 9 * mm, 'ERP42 진단·측정 보고서 | 2026-11-22 | 분석과 미검증 후보를 구분')
    canvas.drawRightString(195 * mm, 9 * mm, str(doc.page))
    canvas.restoreState()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--font-dir', type=Path, default=Path(tempfile.gettempdir()) / 'erp42_report_fonts')
    args = parser.parse_args()
    register_fonts(args.font_dir)
    report, notes, samples = read_data()
    calibration, candidates = write_calibration(report)
    make_figures(report, samples, calibration)
    r = Report()
    pages_one_to_five(r, report, notes)
    pages_six_to_fourteen(r, report, notes, calibration, candidates)
    out = ROOT / 'ERP42_diagnostics_report_2026-11-22.pdf'
    doc = SimpleDocTemplate(str(out), pagesize=A4, rightMargin=15 * mm, leftMargin=15 * mm,
                            topMargin=17 * mm, bottomMargin=18 * mm,
                            title='ERP42 조향·속도 진단 및 측정 보고서', author='ERP42 project / Codex analysis',
                            subject='ROS serial telemetry, steering observations, six speed tests and provisional calibration')
    doc.build(r.story, onFirstPage=footer, onLaterPages=footer)
    print(out)


if __name__ == '__main__':
    main()
