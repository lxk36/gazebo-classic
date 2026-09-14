#!/usr/bin/env python3
"""Extract and compile the actual old/new ODE function with scalar fixtures.

No ROS, Gazebo process, alternative plant, or downloaded experiment data is used.
Original source is read from the exact 11.15.1 commit, not reimplemented here.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
REL = 'deps/opende/src/quickstep_pgs_lcp.cpp'
BASE = 'b22c6e15e52299865b31093b8feebc9ca19e26e8'


def extract(text):
    start = text.index('void quickstep::dxConeFrictionModel(')
    end = text.index('size_t quickstep::EstimatePGS_LCPMemoryRequirements(', start)
    return text[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('cone-receipt'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    old = subprocess.check_output(['git', 'show', BASE+':'+REL], cwd=ROOT)
    assert hashlib.sha1(b'blob '+str(len(old)).encode()+b'\0'+old).hexdigest() == '60ebdf3a2515f348259e7bf1bee800388486eb9f'
    new = (ROOT / REL).read_text()
    assert extract(new).strip() == (HERE/'cone_function.replacement').read_text().strip()
    assert 'inline_position_correction ? params->lambda_erp : NULL' in new
    prefix = (HERE/'cone_stubs.hpp').read_text()
    flags = [os.environ.get('CXX', 'g++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
             '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-g', '-O1']
    env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0')
    with tempfile.TemporaryDirectory() as temporary:
        work = Path(temporary)
        for name, function, suffix in (
            ('old', extract(old.decode()), (HERE/'original_probe.cpp').read_text()),
            ('fixed', extract(new), (HERE/'cone_cases.cpp').read_text()),
        ):
            source, exe = work/(name+'.cpp'), work/name
            source.write_text(prefix+'\n'+function+'\n'+suffix)
            subprocess.run(flags+[str(source), '-o', str(exe)], check=True)
        bad = subprocess.run([str(work/'old')], env=env, text=True, capture_output=True)
        (args.output/'original-asan.log').write_text(bad.stdout+bad.stderr)
        assert bad.returncode != 0 and 'heap-buffer-overflow' in bad.stderr, bad.stderr
        erp = json.loads(subprocess.check_output([str(work/'old'), 'erp'], env=env, text=True))
        zero = json.loads(subprocess.check_output([str(work/'old'), 'erp', 'zero'], env=env, text=True))
        assert erp == {'lower':-36, 'upper':36, 'erp_lower':-36, 'erp_upper':54}
        assert zero == {'lower':0, 'upper':0, 'erp_lower':-999, 'erp_upper':-999}
        fixed = subprocess.run([str(work/'fixed')], env=env, text=True, capture_output=True, check=True)
        result = {'baseline_commit':BASE, 'baseline_heap_overflow_reproduced':True,
                  'baseline_erp':erp, 'baseline_zero_slip':zero,
                  'fixed':json.loads(fixed.stdout),
                  'full_gazebo_build_and_runtime_tested':False}
        (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    (args.output/'production-source.cpp').write_text(new)
    (args.output/'baseline-source.cpp').write_bytes(old)


if __name__ == '__main__':
    main()
