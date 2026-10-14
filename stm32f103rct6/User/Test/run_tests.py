"""Compile and run host-only startup/SDO tests; requires a C11 GCC compiler."""
import argparse
import pathlib
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc', default='gcc', help='Path to host GCC executable')
args = parser.parse_args()
root = pathlib.Path(__file__).resolve().parents[2]
tests = root / 'User/Test'
flags = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-type-limits',
         '-ffunction-sections', '-fdata-sections',
         '-I', str(tests / 'mocks'), '-I', str(root / 'User/Inc'),
         '-I', str(root / 'User/Config')]
suites = {
    'test_motor_startup': ['app_motor_startup.c', 'mid_lisan_ec.c'],
    'test_canopen_sdo': ['mid_canopen.c'],
    'test_position_test': ['app_motor_startup.c', 'mid_lisan_ec.c'],
}
with tempfile.TemporaryDirectory(prefix='chassis_tests_') as directory:
    for name, sources in suites.items():
        exe = pathlib.Path(directory) / (name + '.exe')
        mode_flags = [] if name == 'test_position_test' else ['-DCHASSIS_LINK_POSITION_TEST_ENABLE=0']
        subprocess.run([args.cc, *flags, *mode_flags, str(tests / (name + '.c')),
                        *(str(root / 'User/Src' / source) for source in sources),
                        '-Wl,--gc-sections', '-o', str(exe)], check=True)
        cases = ['normal', 'no_motion', 'lost_ack', 'rejected', 'read_failure', 'halt_failure', 'zero_ramp'] if name == 'test_position_test' else ['']
        for case in cases:
            subprocess.run([str(exe), case], check=True, timeout=30)
