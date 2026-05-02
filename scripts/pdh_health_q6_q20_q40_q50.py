"""Quick electrical-health snapshot for CURIE Q6, Q20, Q40, Q50."""
from logreader.wpilog_reader import read_wpilog

logs = [
    ('Q6',  r'D:\Temp\2026_cmps\FRC_20260430_134357_CURIE_Q6.wpilog'),
    ('Q20', r'D:\Temp\2026_cmps\FRC_20260430_155123_CURIE_Q20.wpilog'),
    ('Q40', r'D:\Temp\2026_cmps\FRC_20260430_195909_CURIE_Q40.wpilog'),
    ('Q50', r'D:\Temp\2026_cmps\FRC_20260430_212244_CURIE_Q50.wpilog'),
]

VPATH = 'NT:/LiveWindow/Ungrouped/PowerDistribution[0]/Voltage'
CPATH = 'NT:/LiveWindow/Ungrouped/PowerDistribution[0]/TotalCurrent'

hdr = (f'{"Match":5s} {"AvgV":>6s} {"MinV":>6s} '
       f'{"<10V%":>7s} {"<9V%":>7s} {"<8V%":>7s} {"<7V%":>7s} '
       f'{"PeakA":>7s} {"AvgA":>6s} {">200A%":>7s} {">250A%":>7s}')
print(hdr)

for tag, path in logs:
    ld = read_wpilog(path)
    v = ld.get_signal(VPATH)
    tc = ld.get_signal(CPATH)
    vv = [float(x.value) for x in v.values]
    cc = [float(x.value) for x in tc.values]

    def pct(f, lst):
        return 100.0 * sum(1 for x in lst if f(x)) / len(lst) if lst else 0.0

    print(
        f'{tag:5s} '
        f'{sum(vv)/len(vv):6.2f} {min(vv):6.2f} '
        f'{pct(lambda x: x < 10.0, vv):7.2f} '
        f'{pct(lambda x: x < 9.0, vv):7.2f} '
        f'{pct(lambda x: x < 8.0, vv):7.2f} '
        f'{pct(lambda x: x < 7.0, vv):7.2f} '
        f'{max(cc):7.1f} {sum(cc)/len(cc):6.1f} '
        f'{pct(lambda x: x > 200, cc):7.2f} '
        f'{pct(lambda x: x > 250, cc):7.2f}'
    )

# Drive-corner peak comparison (Ch0/Ch7/Ch12/Ch19 look like swerve drive)
print()
print('Drive-corner peak amps (Ch0 / Ch7 / Ch12 / Ch19):')
for tag, path in logs:
    ld = read_wpilog(path)
    peaks = []
    for ch in (0, 7, 12, 19):
        sig = ld.get_signal(
            f'NT:/LiveWindow/Ungrouped/PowerDistribution[0]/Chan{ch}')
        peaks.append(max(float(x.value) for x in sig.values) if sig and sig.values else 0.0)
    print(f'  {tag:5s}  Ch0={peaks[0]:6.1f}  Ch7={peaks[1]:6.1f}  '
          f'Ch12={peaks[2]:6.1f}  Ch19={peaks[3]:6.1f}  '
          f'(spread {max(peaks)-min(peaks):.1f} A)')
