import { useSyncExternalStore } from "react";
import { dashboardStore } from "../services/dashboardStore";

export function useDashboard() {
  return useSyncExternalStore(
    dashboardStore.subscribe,
    dashboardStore.getState,
  );
}
