import type { Direcao, MensagemDashboard, Paredes } from "../types/telemetry";

// Adaptador do pacote do firmware (MazeMap::cellPacketJson, HU-00) para as
// mensagens do painel. O firmware manda, a cada célula:
//   { "robot": { "x", "y", "heading": "NORTH" },
//     "cell":  { "x", "y", "visited", "walls": { north, south, east, west } } }
// com a origem (0,0) na célula de largada e y crescendo para o Sul. O painel
// desenha com y crescendo para o Norte, então só o eixo y é invertido
// (y_painel = altura - 1 - y_firmware). Norte, Sul, Leste e Oeste continuam
// os mesmos lados físicos, por isso as paredes não são trocadas.

const HEADING: Record<string, Direcao> = {
  NORTH: "N",
  EAST: "L",
  SOUTH: "S",
  WEST: "O",
};

function ehObjeto(v: unknown): v is Record<string, unknown> {
  return typeof v === "object" && v !== null;
}

function ehParedes(v: unknown): v is Paredes {
  return (
    ehObjeto(v) &&
    typeof v.north === "boolean" &&
    typeof v.south === "boolean" &&
    typeof v.east === "boolean" &&
    typeof v.west === "boolean"
  );
}

// Devolve as mensagens `cell` e `position` do painel, ou null se o dado não
// for um pacote válido do firmware. `altura` vem da mensagem `maze`.
export function adaptarPacoteFirmware(
  dado: unknown,
  altura: number,
): MensagemDashboard[] | null {
  if (!ehObjeto(dado) || !ehObjeto(dado.robot) || !ehObjeto(dado.cell)) return null;
  const { robot, cell } = dado;

  if (
    typeof robot.x !== "number" ||
    typeof robot.y !== "number" ||
    typeof robot.heading !== "string" ||
    typeof cell.x !== "number" ||
    typeof cell.y !== "number" ||
    typeof cell.visited !== "boolean" ||
    !ehParedes(cell.walls)
  ) {
    return null;
  }
  const heading = HEADING[robot.heading];
  if (heading === undefined) return null;

  return [
    {
      type: "cell",
      x: cell.x,
      y: altura - 1 - cell.y,
      visited: cell.visited,
      walls: cell.walls,
    },
    {
      type: "position",
      x: robot.x,
      y: altura - 1 - robot.y,
      heading,
    },
  ];
}
