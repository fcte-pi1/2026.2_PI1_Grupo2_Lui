import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
if(process.platform!=='darwin')throw new Error('O receptor nativo usa o framework IOBluetooth do macOS. Em outros sistemas, use a ponte serial SPP.');
const source=fileURLToPath(new URL('../../firmwareTest/BluetoothTelemetry/tools/receber_mac.m',import.meta.url));
const output=fileURLToPath(new URL('../../firmwareTest/BluetoothTelemetry/tools/receber_mac',import.meta.url));
const result=spawnSync('/usr/bin/clang',['-fobjc-arc','-framework','Foundation','-framework','IOBluetooth',source,'-o',output],{stdio:'inherit'});
if(result.error)throw result.error;
process.exitCode=result.status ?? 1;
