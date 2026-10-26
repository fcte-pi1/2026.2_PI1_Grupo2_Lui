#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

run_coverage() {
  local target="$1"
  shift
  local threshold_flags=(--test-coverage-branches=91)
  if [[ "${1:-}" == "--lines-functions" ]]; then
    threshold_flags+=(--test-coverage-lines=91 --test-coverage-functions=91)
    shift
  fi
  printf '\n\n============================================================\n'
  printf 'Coverage target: %s (thresholds: >90%%)\n' "$target"
  printf '============================================================\n'
  node \
    --experimental-test-coverage \
    --import tsx \
    "${threshold_flags[@]}" \
    --test-coverage-include="$target" \
    --test-coverage-exclude='src/**/__tests__/**' \
    --test "$@"
}

# Cada execução mede um módulo isoladamente. O processo falha imediatamente se
# qualquer alvo ficar em 90% ou menos de cobertura de branches.
run_coverage 'src/services/sessionList.service.ts' \
  src/services/__tests__/sessionList.service.test.ts

run_coverage 'src/controllers/logs.controller.ts' \
  src/controllers/__tests__/logs.controller.test.ts

run_coverage 'src/ws/telemetry.gateway.ts' \
  src/ws/__tests__/telemetry.gateway.test.ts

run_coverage 'src/routes/logs.routes.ts' \
  src/routes/__tests__/logs.routes.test.ts

run_coverage 'src/app.ts' \
  src/__tests__/app.test.ts

run_coverage 'src/server.ts' \
  src/__tests__/server.test.ts

run_coverage 'src/lib/prisma.ts' \
  src/lib/__tests__/prisma.test.ts


# Serviços de domínio: exigem mais de 90% tanto em linhas/funções quanto em branches.
run_coverage 'src/services/telemetry.service.ts' --lines-functions \
  src/services/__tests__/telemetry.service.test.ts

run_coverage 'src/services/sessionNavigation.service.ts' --lines-functions \
  src/services/__tests__/sessionNavigation.service.test.ts

run_coverage 'src/services/logEvento.service.ts' --lines-functions \
  src/services/__tests__/logEvento.service.test.ts

printf '\nTodos os módulos atingiram os limites de cobertura configurados.\n'
