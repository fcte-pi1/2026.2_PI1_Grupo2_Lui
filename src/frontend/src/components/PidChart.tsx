import { useEffect, useRef } from "react";
import {
  Chart,
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale,
  Tooltip,
} from "chart.js";
import type { PontoPid } from "../services/dashboardStore";

Chart.register(LineController, LineElement, PointElement, LinearScale, CategoryScale, Tooltip);

// HU-09 CA-02: gráfico contínuo do erro do controle PID ao longo do tempo
export function PidChart({ pontos }: { pontos: PontoPid[] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const chartRef = useRef<Chart | null>(null);

  useEffect(() => {
    if (!canvasRef.current) return;
    chartRef.current = new Chart(canvasRef.current, {
      type: "line",
      data: {
        labels: [],
        datasets: [
          {
            label: "Erro PID",
            data: [],
            borderColor: "#e11d48",
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.3,
          },
        ],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: { ticks: { maxTicksLimit: 6 } },
          y: { title: { display: true, text: "erro" } },
        },
      },
    });
    return () => chartRef.current?.destroy();
  }, []);

  useEffect(() => {
    const chart = chartRef.current;
    if (!chart || !pontos.length) return;
    const t0 = pontos[0].timestamp;
    chart.data.labels = pontos.map((p) => `${((p.timestamp - t0) / 1000).toFixed(1)}s`);
    chart.data.datasets[0].data = pontos.map((p) => p.erro);
    chart.update("none");
  }, [pontos]);

  return (
    <div className="relative h-48">
      <canvas ref={canvasRef} />
    </div>
  );
}
