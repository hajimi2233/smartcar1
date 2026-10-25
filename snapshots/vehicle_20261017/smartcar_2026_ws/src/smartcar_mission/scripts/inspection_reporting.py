#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Ordered channel descriptions and asynchronous cached audio; Python 2/3."""
from __future__ import unicode_literals
import json
import os
import re
import subprocess
import threading
import time
import uuid
import wave
try:
    import Queue as queue
except ImportError:
    import queue

KINDS = {'1': '火源', '2': '水源', '3': '行人', '4': '道路'}
STATES = {'A': '正常', 'B': '异常'}
try:
    text_type = unicode
except NameError:
    text_type = str


def terminate(process):
    if process is not None and process.poll() is None:
        try: process.terminate()
        except OSError: pass


def parse_description(text, count=10):
    text = re.sub(r'\s+', '', text).upper()
    if not text or not re.match(r'^(?:[AB][1-4])+$', text):
        raise ValueError('Expected A/B + category 1..4 for each channel')
    tokens = [text[i:i+2] for i in range(0, len(text), 2)]
    if len(tokens) != count:
        raise ValueError('Expected %d channels in input order; got %d' % (count, len(tokens)))
    reports = {}
    for i, token in enumerate(tokens, 1):
        reports['inspect_%d' % i] = dict(channel_number=i, condition=token[0],
                                       object_kind=token[1], speech_text=STATES[token[0]]+KINDS[token[1]])
    legacy = ''.join(token[0]+str(i) for i, token in enumerate(tokens, 1))
    return legacy, reports


def annotate_plan(raw, reports):
    rows = [json.loads(line) for line in raw.decode('utf-8').splitlines() if line.strip()]
    seen = set()
    for row in rows:
        label = row.get('target_id')
        if label in reports:
            if row.get('inspection_type') != reports[label]['condition']:
                raise ValueError('Planner inspection type differs from description')
            row.update(reports[label]); seen.add(label)
    if seen != set(reports):
        raise ValueError('Missing described inspection channels in plan')
    return ('\n'.join(json.dumps(r, ensure_ascii=False) for r in rows)+'\n').encode('utf-8')


def load_reports(path):
    reports = {}
    with open(path, 'rb') as stream:
        for line in stream:
            if not line.strip(): continue
            row = json.loads(line.decode('utf-8'))
            if 'speech_text' not in row: continue
            label = row.get('target_id', '')
            kind, state = row.get('object_kind'), row.get('condition')
            number = row.get('channel_number')
            if (kind not in KINDS or state not in STATES or isinstance(number, bool)
                    or number not in range(1, 11) or label != 'inspect_%d' % number
                    or row['speech_text'] != STATES[state]+KINDS[kind]
                    or row.get('inspection_type') != state or label in reports):
                raise ValueError('Invalid inspection speech metadata')
            reports[label] = dict((k, row[k]) for k in
                                 ('channel_number', 'condition', 'object_kind', 'speech_text'))
    if reports and len(reports) != 10:
        raise ValueError('Speech metadata must describe all ten channels')
    return reports


def cached_audio(workspace):
    """Use validated existing WAVs only; never synthesize during navigation."""
    directory = os.path.join(workspace, 'data', 'audio', 'tts_cache')
    expected = set(a+b for a in STATES.values() for b in KINDS.values())
    selected = {}
    if not os.path.isdir(directory): return {}
    try: names = sorted(os.listdir(directory))
    except OSError: return {}
    for name in names:
        if not re.match(r'^[0-9a-f]{64}\.json$', name): continue
        path = os.path.join(directory, name[:-5]+'.wav')
        try:
            with open(os.path.join(directory, name), 'rb') as f:
                data = json.loads(f.read().decode('utf-8'))
            text = data.get('text')
            if text not in expected or data.get('speed') != 1.: continue
            if os.path.realpath(path) != path: continue
            wav = wave.open(path, 'rb')
            try:
                duration = wav.getnframes()/float(wav.getframerate())
                if wav.getsampwidth() != 2 or wav.getnchannels() != 1 or not 0 < duration <= 15.: continue
            finally:
                wav.close()
            selected[text] = (path, duration)
        except (IOError, OSError, ValueError, EOFError, wave.Error):
            continue
    return selected


class ArrivalSpeaker(object):
    def __init__(self, audio, notify, enabled=True):
        self.audio, self.notify, self.enabled = audio, notify, enabled
        self.queue = queue.Queue(32)
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.generation = 0; self.process = None; self.seen = set(); self.run_id = None
        self.thread = threading.Thread(target=self.work)
        self.thread.daemon = True; self.thread.start()

    def begin_run(self):
        with self.lock:
            self.generation += 1; self.seen.clear(); self.run_id = uuid.uuid4().hex
            terminate(self.process)
            while True:
                try: self.queue.get_nowait()
                except queue.Empty: break

    def enqueue(self, label, report):
        with self.lock:
            if label in self.seen or self.stop.is_set(): return
            self.seen.add(label)
            event = dict(report, target_id=label, run_id=self.run_id, event='ARRIVED')
            self.notify(event)
            if not self.enabled: return
            try: self.queue.put_nowait((self.generation, event))
            except queue.Full: self.notify(dict(event, event='AUDIO_FAILED', reason='QUEUE_FULL'))

    def close(self):
        self.stop.set()
        with self.lock:
            self.generation += 1
            terminate(self.process)

    def work(self):
        while not self.stop.is_set():
            try: generation, event = self.queue.get(timeout=.1)
            except queue.Empty: continue
            process = None
            try:
                with self.lock:
                    if generation != self.generation or self.stop.is_set(): continue
                    text = event['speech_text']
                    if text not in self.audio: raise ValueError('MISSING_CACHED_AUDIO')
                    path, duration = self.audio[text]
                    process = subprocess.Popen(['paplay', path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.process = process
                # Linux elapsed real time, unaffected by wall-clock corrections.
                deadline = os.times()[4]+duration+5.
                while process.poll() is None:
                    if self.stop.is_set() or generation != self.generation or os.times()[4] >= deadline:
                        terminate(process); time.sleep(.1)
                        if process.poll() is None: process.kill()
                        raise RuntimeError('PLAYBACK_CANCELLED_OR_TIMEOUT')
                    self.stop.wait(.05)
                out, error = process.communicate()
                if process.returncode:
                    raise RuntimeError(error.decode('utf-8', 'replace')[-500:])
                self.notify(dict(event, event='AUDIO_FINISHED'))
            except Exception as exc:
                self.notify(dict(event, event='AUDIO_FAILED', reason=text_type(exc)))
            finally:
                if process is not None:
                    if process.poll() is None: process.kill()
                    process.wait()
                    for pipe in (process.stdout, process.stderr):
                        if pipe is not None and not pipe.closed: pipe.close()
                with self.lock:
                    if self.process is process: self.process = None
