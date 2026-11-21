ERP42 diagnostics and measurement report (2026-11-22, Asia/Seoul)

Open ERP42_diagnostics_report_2026-11-22.pdf for the Korean report.
The report includes the serial data path, steering observations, all six
2026-11-21 speed trials, figures, provisional calibration, and next tests.

Applied driver changes are recorded in commit:
  3e2363caefd859ed45cb3b2796971e261d21c75c
  - feedback_raw: raw speed and signed steering counts.
  - feedback_bytes: accepted 18-byte RX frame.
  - signed steering decoding includes the 0x8000 boundary.

The candidate RX divisor 11.65 and steering offsets are NOT applied.
Current driver speed/steer conversion remains integer division by 10/71.
The reported input/18.71 relation only fits the average speed of the
14.2 m starts from rest; it is not a steady-state speed conversion.

Reproduce the numerical analysis (ROS Noetic + Python 3 + NumPy/Matplotlib):
  source /opt/ros/noetic/setup.bash
  /usr/bin/python3 analysis/analyze_bags.py --bag-dir /path/to/bags

All six expected filenames are listed in analysis/analyze_bags.py.
Original bag files are excluded by the repository's existing .gitignore.
The separate downloadable ZIP contains them in bags/, so within that ZIP:
  /usr/bin/python3 analysis/analyze_bags.py --bag-dir bags

Rebuild the PDF (Python 3 + ReportLab + NumPy/Matplotlib):
  /usr/bin/python3 build_report.py
Nanum Gothic fonts and their SIL OFL license are downloaded from google/fonts
and cached under the system temporary directory if missing. The PDF embeds
the used Korean glyphs. --font-dir can select a pre-populated font directory.

Data and assumptions:
  measurement_notes.json       User observations and confirmed conditions.
  analysis/speed_summary.csv    Per-trial metrics and approximate windows.
  analysis/speed_samples.csv    Aligned raw, feedback, bytes, and commands.
  analysis/speed_analysis.json  Detailed checks, assumptions, source bag hashes.
  analysis/calibration_*       Candidate divisors and timing sensitivity.
  figures/                     Publication figures used in the PDF.
  analysis/*.png / *.svg        Original analysis figures.

All analysis is offline. Scripts do not publish ROS commands or operate the
vehicle. Actual hardware, model-specific units, low-speed accuracy, and the
reported 40-second initial-motion delay remain outside this validation.
