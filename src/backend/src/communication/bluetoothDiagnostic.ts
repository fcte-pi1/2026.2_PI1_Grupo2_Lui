export interface BluetoothDiagnostic {
  type: "bt_telemetry_test";
  seq: number;
  uptime_ms: number;
  heap_bytes: number;
  motors_enabled: false;
  ultrasonic_enabled: false;
  imu: { valid: boolean; yaw_deg: number | null; gyro_z_dps: number | null;
    accel_g: { x: number | null; y: number | null; z: number | null } };
  received_at_ms: number;
}
export class BluetoothDiagnosticError extends Error {}
export function record(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}
function finite(value: unknown): number {
  if (typeof value !== "number" || !Number.isFinite(value)) throw new BluetoothDiagnosticError("Medida IMU inválida");
  return value;
}
function counter(value: unknown): number {
  const number=finite(value);
  if (!Number.isInteger(number) || number < 0 || number > 0xffffffff) throw new BluetoothDiagnosticError("Contador Bluetooth inválido");
  return number;
}
export function validateBluetoothDiagnostic(packet: Record<string, unknown>, now=Date.now()): BluetoothDiagnostic {
  if (packet.motors_enabled !== false || packet.ultrasonic_enabled !== false)
    throw new BluetoothDiagnosticError("O diagnóstico exige motores e ultrassons desabilitados");
  const imu=packet.imu;
  if (!record(imu) || typeof imu.valid !== "boolean" || !record(imu.accel_g))
    throw new BluetoothDiagnosticError("Pacote IMU inválido");
  const measure=(value: unknown) => imu.valid ? finite(value) : null;
  return { type:"bt_telemetry_test", seq:counter(packet.seq), uptime_ms:counter(packet.uptime_ms),
    heap_bytes:counter(packet.heap_bytes), motors_enabled:false, ultrasonic_enabled:false,
    imu:{valid:imu.valid,yaw_deg:measure(imu.yaw_deg),gyro_z_dps:measure(imu.gyro_z_dps),
      accel_g:{x:measure(imu.accel_g.x),y:measure(imu.accel_g.y),z:measure(imu.accel_g.z)}}, received_at_ms:now };
}
