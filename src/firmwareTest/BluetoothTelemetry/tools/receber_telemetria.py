#!/usr/bin/env python3
"""Receive real ESP32 test telemetry over a macOS Bluetooth SPP serial port."""
import argparse
import json
import time
import uuid
from pathlib import Path


def receive_frames(transport, duration, capture, minimum_frames, clock=time.monotonic):
    token = 'pi1-' + uuid.uuid4().hex[:12]
    transport.write(('PING ' + token + '\n').encode('ascii'))
    started = clock()
    pending = bytearray()
    received = valid_imu = bad = gaps = 0
    previous_seq = None
    pong = False
    while clock() - started < duration:
        chunk = transport.read(512)
        if not chunk:
            continue
        pending.extend(chunk)
        if len(pending) > 65536:
            raise ValueError('Frame maior que 64 KiB; interrompendo recepcao.')
        while b'\n' in pending:
            line, _, rest = pending.partition(b'\n')
            pending = bytearray(rest)
            if not line.strip():
                continue
            try:
                packet = json.loads(line)
                if not isinstance(packet, dict):
                    raise ValueError('Esperado objeto JSON')
                capture.write(json.dumps(packet, ensure_ascii=False) + '\n')
                capture.flush()
                if packet.get('type') == 'pong':
                    pong = packet.get('token') == token or pong
                if packet.get('type') != 'bt_telemetry_test':
                    continue
                if packet.get('motors_enabled') is not False or packet.get('ultrasonic_enabled') is not False:
                    raise RuntimeError('Estado inesperado: o teste exige motores e ultrassons desabilitados.')
                seq = packet['seq']
                if type(seq) is not int or type(packet['uptime_ms']) is not int:
                    raise ValueError('Sequencia/tempo invalidos')
                if previous_seq is not None:
                    if seq <= previous_seq:
                        raise ValueError('Sequencia repetida/reiniciada durante a captura')
                    gaps += seq - previous_seq - 1
                previous_seq = seq
                received += 1
                imu = packet.get('imu', {})
                if imu.get('valid') is True:
                    valid_imu += 1
                if received <= 3 or received % 10 == 0:
                    print(f"seq={seq} uptime={packet['uptime_ms']}ms IMU={imu.get('valid')} "
                          f"yaw={imu.get('yaw_deg')} giroZ={imu.get('gyro_z_dps')} "
                          f"motores=OFF sonar=OFF", flush=True)
            except (json.JSONDecodeError, KeyError, ValueError, UnicodeDecodeError) as exc:
                bad += 1
                print(f'Frame rejeitado: {exc}', flush=True)
    summary = {'frames': received, 'imu_valid_frames': valid_imu, 'invalid_frames': bad,
               'sequence_gaps': gaps, 'ping_pong_ok': pong,
               'duration_s': round(clock() - started, 2)}
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    if received < minimum_frames or bad or not pong:
        raise RuntimeError('Validacao incompleta: conferir quadros recebidos e resposta PING/PONG.')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/cu.MicroMouse_BT_Test')
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--min-frames', type=int, default=8)
    parser.add_argument('--capture', type=Path, default=Path('telemetria-bluetooth.jsonl'))
    args = parser.parse_args()
    if not args.port or not 1 <= args.seconds <= 300 or args.min_frames < 1:
        parser.error('Usar duracao de 1 a 300 segundos e minimo positivo de quadros.')
    import serial
    connection = serial.Serial()
    connection.port = args.port
    connection.baudrate = 115200  # API serial; does not set the Bluetooth radio bitrate.
    connection.timeout = 0.2
    connection.write_timeout = 3
    connection.dtr = False
    connection.rts = False
    connection.open()
    print(f'Recebendo da porta Bluetooth {args.port}', flush=True)
    try:
        with args.capture.open('w', encoding='utf-8') as capture:
            receive_frames(connection, args.seconds, capture, args.min_frames)
    finally:
        connection.close()


if __name__ == '__main__':
    main()
