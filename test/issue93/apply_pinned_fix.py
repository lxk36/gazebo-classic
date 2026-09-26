#!/usr/bin/env python3
"""One-time, hash-guarded source edit for Gazebo 11.15.1's bundled ODE.

This edits the actual production .cpp, not a runtime override or substitute
plant. It is idempotent only for the exact replacement committed beside it.
"""
from hashlib import sha1
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / 'deps/opende/src/quickstep_pgs_lcp.cpp'
OLD_BLOB = '60ebdf3a2515f348259e7bf1bee800388486eb9f'
START = 'void quickstep::dxConeFrictionModel('
END = 'size_t quickstep::EstimatePGS_LCPMemoryRequirements('


def main():
    raw = PATH.read_bytes()
    text = raw.decode('utf-8')
    fixed = (Path(__file__).parent / 'cone_function.replacement').read_text().rstrip() + '\n\n'
    start, end = text.index(START), text.index(END)
    if text[start:end] == fixed and 'inline_position_correction ? params->lambda_erp : NULL' in text:
        print('Production source already contains the pinned fix')
        return
    blob = sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest()
    if blob != OLD_BLOB:
        raise SystemExit(f'Refusing unreviewed ODE source blob {blob}; expected {OLD_BLOB}')
    text = text[:start] + fixed + text[end:]
    for old, new in (
        ('dRealPtr rhs_erp             = params->rhs_erp;',
         'dRealPtr rhs_erp             = inline_position_correction ? params->rhs_erp : NULL;'),
        ('dRealMutablePtr caccel_erp   = params->caccel_erp;',
         'dRealMutablePtr caccel_erp   = inline_position_correction ? params->caccel_erp : NULL;'),
        ('dRealMutablePtr lambda_erp   = params->lambda_erp;',
         'dRealMutablePtr lambda_erp   = inline_position_correction ? params->lambda_erp : NULL;'),
    ):
        if text.count(old) != 1:
            raise SystemExit(f'Expected one production pointer initialization: {old}')
        text = text.replace(old, new)
    PATH.write_text(text)
    print('Patched the actual bundled ODE production source; no parameter or friction-law change')


if __name__ == '__main__':
    main()
