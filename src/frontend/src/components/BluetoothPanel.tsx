import { useEffect, useState } from "react";
import type { BluetoothDiagnostic, BluetoothLinkStatus } from "../types/telemetry";

const format=(value:number|null|undefined,unit:string,decimals=3)=>value==null ? "Sem leitura" : `${value.toFixed(decimals)} ${unit}`;
export function BluetoothPanel({data,link,compact=false}:{data:BluetoothDiagnostic|null;link:BluetoothLinkStatus|null;compact?:boolean}) {
  const [now,setNow]=useState(Date.now);
  useEffect(()=>{const timer=setInterval(()=>setNow(Date.now()),500);return()=>clearInterval(timer);},[]);
  const age=data ? Math.max(0,now-data.received_at_ms) : null;
  const recent=age!==null&&age<2500;
  const connected=link?.status==="connected";
  const status=connected&&recent ? "Recebendo dados reais" : connected&&data ? "Sem dados recentes" : link?.status==="connecting" ? "Conectando à ESP32" : "Aguardando Bluetooth";
  const metrics=[
    ["Yaw relativo",format(data?.imu.yaw_deg,"°")],
    ["Giro no eixo Z",format(data?.imu.gyro_z_dps,"°/s")],
    ["Aceleração X",format(data?.imu.accel_g.x,"g")],
    ["Aceleração Y",format(data?.imu.accel_g.y,"g")],
    ["Aceleração Z",format(data?.imu.accel_g.z,"g")],
    ["Memória livre",data ? `${(data.heap_bytes/1024).toFixed(1)} KiB` : "Sem leitura"],
  ];
  return <section aria-label="Monitor Bluetooth" className={compact ? "mb-4" : "rounded-2xl border border-slate-200 bg-white p-6 dark:border-slate-800 dark:bg-slate-900"}>
    <div className="flex flex-wrap items-center justify-between gap-3">
      <div><h2 className="text-xl font-semibold">Bluetooth ao vivo</h2><p className="mt-1 text-sm text-slate-500">Teste parado · dados da ESP32</p></div>
      <p role="status" className={`rounded-full px-3 py-1 text-sm font-medium ${connected&&recent ? "bg-emerald-100 text-emerald-800" : "bg-amber-100 text-amber-900"}`}>{status}</p>
    </div>
    <p className="mt-4 rounded-lg bg-slate-50 p-3 text-sm dark:bg-slate-800">Motores desabilitados · ultrassons sem disparos · nenhuma corrida sendo registrada.</p>
    <dl className={`mt-5 grid gap-4 ${compact ? "sm:grid-cols-2 lg:grid-cols-1 xl:grid-cols-2" : "sm:grid-cols-2 lg:grid-cols-3"} ${data&&!recent ? "opacity-60" : ""}`}>
      {metrics.map(([label,value])=><div key={label} className="rounded-xl border border-slate-200 p-4 dark:border-slate-700"><dt className="text-sm text-slate-500">{label}</dt><dd className="mt-1 text-xl font-semibold" data-testid={label==="Yaw relativo"?"bluetooth-yaw":undefined}>{value}</dd></div>)}
    </dl>
    <div className="mt-4 flex flex-wrap gap-x-6 gap-y-1 text-sm text-slate-500">
      <span>Pacote: <strong data-testid="bluetooth-sequence">{data?.seq ?? "—"}</strong></span>
      <span>Tempo ligado: {data ? `${(data.uptime_ms/1000).toFixed(1)} s` : "—"}</span>
      <span>Última leitura: {age===null ? "aguardando" : `${(age/1000).toFixed(1)} s atrás`}</span>
    </div>
    {data&&!data.imu.valid&&<p role="alert" className="mt-3 text-amber-700">A IMU não forneceu uma leitura válida. As medidas estão indisponíveis.</p>}
    <p className="mt-4 text-sm text-slate-500">O yaw é relativo, com compensação de repouso no firmware de teste. Giros muito lentos (abaixo de 0,25°/s) podem ser tratados como repouso. Distância das paredes e bateria não são medidas neste teste.</p>
  </section>;
}
