#!/usr/bin/env python3
"""
VoiceBeam Receiver
==================
Receives audio beamed from the VoiceBeam web app over Bluetooth.

Two modes:
  --mode rfcomm   Classic Bluetooth serial (SPP) — easiest, works on most devices
  --mode ble      BLE GATT server — matches Web Bluetooth API used by the app

Requirements:
  pip install PyBluez bleak soundfile numpy pyaudio

Usage:
  python voicebeam_receiver.py --mode rfcomm
  python voicebeam_receiver.py --mode ble
  python voicebeam_receiver.py --mode rfcomm --output ./recordings/
"""

import argparse
import asyncio
import io
import os
import struct
import time
import wave
from datetime import datetime
from pathlib import Path


# ─── Shared helpers ────────────────────────────────────────────────────────────

def make_output_path(output_dir: str, ext: str = "webm") -> Path:
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return out / f"voicebeam_{ts}.{ext}"


def save_audio(data: bytes, output_dir: str) -> Path:
    """
    Save raw bytes from the browser (audio/webm;codecs=opus).
    ffmpeg can convert this to wav/mp3 afterwards:
      ffmpeg -i voicebeam_YYYYMMDD_HHMMSS.webm out.wav
    """
    path = make_output_path(output_dir, ext="webm")
    path.write_bytes(data)
    print(f"  ✓ Saved: {path}  ({len(data):,} bytes)")
    return path


def play_audio(data: bytes) -> None:
    """Optional live playback via pyaudio (best-effort)."""
    try:
        import pyaudio
        import subprocess, tempfile, threading

        # Write to temp file and decode with ffmpeg → PCM pipe
        with tempfile.NamedTemporaryFile(suffix=".webm", delete=False) as f:
            f.write(data)
            tmp = f.name

        proc = subprocess.Popen(
            ["ffmpeg", "-i", tmp, "-f", "s16le", "-ar", "44100", "-ac", "1", "-"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        pcm, _ = proc.communicate()
        os.unlink(tmp)

        pa = pyaudio.PyAudio()
        stream = pa.open(format=pyaudio.paInt16, channels=1, rate=44100, output=True)
        stream.write(pcm)
        stream.stop_stream()
        stream.close()
        pa.terminate()
    except Exception as e:
        print(f"  [playback skipped: {e}]")


# ─── Protocol framing ──────────────────────────────────────────────────────────
#
# Both RFCOMM and BLE use the same simple framing so the app and receiver
# agree on message boundaries:
#
#   [ 4-byte magic "VBAM" ][ 4-byte uint32 payload_length ][ payload bytes ]
#
# The web app sends raw audio/webm blobs. Wrap them in this frame before
# sending. The receiver reassembles frames and saves each blob as a file.

MAGIC = b"VBAM"
HEADER_SIZE = 8   # 4 magic + 4 length


def encode_frame(data: bytes) -> bytes:
    return MAGIC + struct.pack(">I", len(data)) + data


def _recv_exact(conn, n: int) -> bytes | None:
    """Read exactly n bytes from a blocking socket-like object."""
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def recv_frame(conn) -> bytes | None:
    """Read one framed message. Returns payload or None on disconnect."""
    header = _recv_exact(conn, HEADER_SIZE)
    if header is None:
        return None
    if not header.startswith(MAGIC):
        print(f"  [bad magic: {header[:4]!r}, flushing]")
        return None
    (length,) = struct.unpack(">I", header[4:])
    if length > 50 * 1024 * 1024:   # 50 MB sanity cap
        print(f"  [implausible payload size {length}, dropping]")
        return None
    return _recv_exact(conn, length)


# ─── Mode 1: Classic Bluetooth RFCOMM (SPP) ────────────────────────────────────

RFCOMM_CHANNEL = 4    # match this in the web app / paired device
RFCOMM_UUID    = "00001101-0000-1000-8000-00805F9B34FB"   # SPP


def run_rfcomm(output_dir: str, play: bool) -> None:
    """
    Advertise an RFCOMM server and accept audio from VoiceBeam.

    On the web app side, replace the simulated beamRecording() with:
      const service = await device.gatt.getPrimaryService('00001101-0000-1000-8000-00805f9b34fb');
      const char    = await service.getCharacteristic(...);
      await char.writeValue(encode_frame(audioBlob));
    """
    try:
        import bluetooth  # PyBluez
    except ImportError:
        print("PyBluez not installed. Run:  pip install PyBluez")
        return

    server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server.bind(("", RFCOMM_CHANNEL))
    server.listen(1)

    bluetooth.advertise_service(
        server,
        "VoiceBeam Receiver",
        service_id=RFCOMM_UUID,
        service_classes=[RFCOMM_UUID, bluetooth.SERIAL_PORT_CLASS],
        profiles=[bluetooth.SERIAL_PORT_PROFILE],
    )

    print(f"[RFCOMM] Listening on channel {RFCOMM_CHANNEL}  (UUID {RFCOMM_UUID})")
    print("[RFCOMM] Pair this machine with your phone/browser device, then start beaming.\n")

    try:
        while True:
            print("Waiting for connection …")
            conn, addr = server.accept()
            print(f"  ← Connected: {addr}")

            try:
                while True:
                    payload = recv_frame(conn)
                    if payload is None:
                        print("  Connection closed.")
                        break
                    print(f"  ← Received frame  {len(payload):,} bytes")
                    path = save_audio(payload, output_dir)
                    if play:
                        play_audio(payload)
            except OSError as e:
                print(f"  [socket error: {e}]")
            finally:
                conn.close()
    except KeyboardInterrupt:
        print("\n[RFCOMM] Stopped.")
    finally:
        server.close()


# ─── Mode 2: BLE GATT server (Web Bluetooth compatible) ───────────────────────
#
# Web Bluetooth talks GATT. We expose a custom service + characteristic.
# The browser does:
#   const svc  = await device.gatt.getPrimaryService(VOICEBEAM_SERVICE_UUID);
#   const char = await svc.getCharacteristic(VOICEBEAM_CHAR_UUID);
#   await char.writeValueWithResponse(encode_frame(audioBlob));
#
# Because bleak (Python) can only act as a GATT *client* on Linux/macOS,
# this mode uses BlueZ D-Bus directly on Linux.  For other platforms it
# falls back to printing instructions for nRF Connect / similar tools.

VOICEBEAM_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
VOICEBEAM_CHAR_UUID    = "12345678-1234-5678-1234-56789abcdef1"


async def run_ble(output_dir: str, play: bool) -> None:
    """BLE GATT peripheral mode (Linux/BlueZ)."""

    # Try bless for cross-platform BLE peripheral support
    try:
        from bless import BlessServer, BlessGATTCharacteristicProperties, BlessGATTCharacteristicPermissions
    except ImportError:
        print("bless not installed. Run:  pip install bless")
        print("\nAlternatively, use --mode rfcomm for classic Bluetooth.")
        return

    loop = asyncio.get_event_loop()
    receive_buf: dict[str, bytearray] = {"data": bytearray()}
    frame_q: asyncio.Queue = asyncio.Queue()

    def write_request(characteristic, value: bytes) -> None:
        """Called each time the browser writes a chunk."""
        receive_buf["data"].extend(value)
        buf = bytes(receive_buf["data"])

        # Try to extract complete frames
        while len(buf) >= HEADER_SIZE:
            if not buf.startswith(MAGIC):
                # Re-sync: skip one byte
                buf = buf[1:]
                continue
            (length,) = struct.unpack(">I", buf[4:8])
            total = HEADER_SIZE + length
            if len(buf) < total:
                break   # incomplete frame, wait for more chunks
            payload = buf[HEADER_SIZE:total]
            buf = buf[total:]
            loop.call_soon_threadsafe(frame_q.put_nowait, payload)

        receive_buf["data"] = bytearray(buf)

    server = BlessServer(name="VoiceBeam")
    server.read_request_func  = lambda c, **kw: b""
    server.write_request_func = write_request

    await server.add_new_service(VOICEBEAM_SERVICE_UUID)
    await server.add_new_characteristic(
        VOICEBEAM_SERVICE_UUID,
        VOICEBEAM_CHAR_UUID,
        BlessGATTCharacteristicProperties.write,
        None,
        BlessGATTCharacteristicPermissions.writeable,
    )

    await server.start()
    print(f"[BLE] Advertising as 'VoiceBeam'")
    print(f"  Service UUID : {VOICEBEAM_SERVICE_UUID}")
    print(f"  Char UUID    : {VOICEBEAM_CHAR_UUID}")
    print("\nIn your VoiceBeam app, scan for 'VoiceBeam' and start beaming.\n")

    try:
        while True:
            payload = await frame_q.get()
            print(f"  ← Received frame  {len(payload):,} bytes")
            path = save_audio(payload, output_dir)
            if play:
                play_audio(payload)
    except asyncio.CancelledError:
        pass
    finally:
        await server.stop()
        print("\n[BLE] Stopped.")


# ─── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="VoiceBeam Receiver — accept audio from the VoiceBeam web app over Bluetooth"
    )
    parser.add_argument(
        "--mode", choices=["rfcomm", "ble"], default="rfcomm",
        help="rfcomm = classic Bluetooth SPP (default) | ble = BLE GATT"
    )
    parser.add_argument(
        "--output", default="./recordings",
        help="Directory to save received audio files (default: ./recordings)"
    )
    parser.add_argument(
        "--play", action="store_true",
        help="Play received audio immediately via ffmpeg + pyaudio (requires ffmpeg on PATH)"
    )
    args = parser.parse_args()

    print("=" * 55)
    print("  VoiceBeam Receiver")
    print(f"  Mode   : {args.mode.upper()}")
    print(f"  Output : {args.output}")
    print(f"  Play   : {args.play}")
    print("=" * 55 + "\n")

    if args.mode == "rfcomm":
        run_rfcomm(args.output, args.play)
    else:
        asyncio.run(run_ble(args.output, args.play))


if __name__ == "__main__":
    main()