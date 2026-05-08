#!/usr/bin/env python3
"""Check faster-whisper loading and optional microphone transcription."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Load faster-whisper like face_display_node and optionally '
            'record/transcribe microphone audio.'
        )
    )
    parser.add_argument('--model-size', default='base')
    parser.add_argument(
        '--model-path',
        default='',
        help='Optional local faster-whisper/CTranslate2 model directory.',
    )
    parser.add_argument('--device', default='auto', help='auto, cuda, or cpu')
    parser.add_argument(
        '--compute-type',
        default='auto',
        help='auto, int8_float16, float16, int8, etc.',
    )
    parser.add_argument(
        '--local-files-only',
        action='store_true',
        help='Do not download model files; require them to already be cached.',
    )
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--sample-rate', type=int, default=16000)
    parser.add_argument(
        '--skip-mic',
        action='store_true',
        help='Only check model loading; do not record audio.',
    )
    return parser.parse_args()


def get_load_attempts(device: str, compute_type: str) -> list[tuple[str, str]]:
    requested_device = device.strip().lower()
    requested_compute_type = compute_type.strip().lower()

    if requested_compute_type in ('', 'auto'):
        gpu_compute_type = 'int8_float16'
        cpu_compute_type = 'int8'
    else:
        gpu_compute_type = requested_compute_type
        cpu_compute_type = requested_compute_type

    if requested_device in ('', 'auto'):
        return [('cuda', gpu_compute_type), ('cpu', cpu_compute_type)]

    selected_compute_type = (
        cpu_compute_type if requested_device == 'cpu' else gpu_compute_type
    )
    attempts = [(requested_device, selected_compute_type)]
    if requested_device != 'cpu':
        attempts.append(('cpu', 'int8'))
    return attempts


def load_model(args: argparse.Namespace):
    from faster_whisper import WhisperModel

    model_path = Path(args.model_path).expanduser() if args.model_path else None
    model_source = (
        str(model_path)
        if model_path is not None and model_path.is_dir()
        else args.model_size
    )

    last_error: Exception | None = None
    for device, compute_type in get_load_attempts(args.device, args.compute_type):
        print(
            f'Loading faster-whisper model "{model_source}" on {device} '
            f'with {compute_type} compute...'
        )
        started = time.monotonic()
        try:
            model = WhisperModel(
                model_source,
                device=device,
                compute_type=compute_type,
                local_files_only=args.local_files_only,
            )
        except Exception as exc:
            last_error = exc
            print(f'  failed: {exc!r}')
            continue

        elapsed = time.monotonic() - started
        print(f'Loaded OK on {device} with {compute_type} in {elapsed:.1f}s.')
        return model, device, compute_type

    raise RuntimeError('Could not load faster-whisper model.') from last_error


def record_audio(seconds: float, sample_rate: int) -> np.ndarray:
    import sounddevice as sd

    frames = max(1, int(seconds * sample_rate))
    print(f'Recording {seconds:.1f}s at {sample_rate} Hz. Speak now.')
    audio = sd.rec(frames, samplerate=sample_rate, channels=1, dtype='float32')
    sd.wait()
    return np.ascontiguousarray(audio.squeeze())


def transcribe(model, audio: np.ndarray) -> str:
    print('Transcribing...')
    started = time.monotonic()
    segments, info = model.transcribe(
        audio,
        language='en',
        beam_size=5,
        vad_filter=True,
    )
    text = ''.join(segment.text for segment in segments).strip()
    elapsed = time.monotonic() - started
    language = getattr(info, 'language', 'unknown')
    probability = getattr(info, 'language_probability', 0.0)
    print(
        f'Transcribed in {elapsed:.1f}s '
        f'(language={language}, probability={probability:.2f}).'
    )
    return text


def main() -> int:
    args = parse_args()

    try:
        model, device, compute_type = load_model(args)
    except Exception as exc:
        print(f'\nModel check failed: {exc!r}', file=sys.stderr)
        return 1

    if args.skip_mic:
        print(f'\nPASS: faster-whisper loaded on {device} with {compute_type}.')
        return 0

    try:
        audio = record_audio(args.seconds, args.sample_rate)
        text = transcribe(model, audio)
    except Exception as exc:
        print(f'\nMic/transcription check failed: {exc!r}', file=sys.stderr)
        return 2

    print('\nRecognized text:')
    print(text or '<no speech detected>')
    print(f'\nPASS: faster-whisper ran on {device} with {compute_type}.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
