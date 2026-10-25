from __future__ import print_function
import hashlib
import json
import os
import sys

root = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(root, 'MANIFEST.json'), 'rb') as stream:
    rows = json.loads(stream.read().decode('utf-8'))
failed = []
for row in rows:
    path = os.path.join(root, *row['path'].split('/'))
    try:
        with open(path, 'rb') as stream:
            actual = hashlib.sha256(stream.read()).hexdigest()
        if actual != row['sha256']:
            failed.append(row['path'])
    except IOError:
        failed.append(row['path'])
for path in failed:
    print('MISSING_OR_CHANGED: ' + path)
print('Checked %d archived files; failures: %d' % (len(rows), len(failed)))
sys.exit(1 if failed else 0)
