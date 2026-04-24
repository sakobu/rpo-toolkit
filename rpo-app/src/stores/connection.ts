import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

import type { ServerError } from '@/schemas/wsProtocol';

export type ConnectionStatus =
  | { status: 'DISCONNECTED' }
  | { status: 'CONNECTING' }
  | { status: 'IDLE' }
  | { status: 'BUSY'; inFlightCount: number }
  | { status: 'RECONNECTING'; attempt: number; lastError?: ServerError };

type ConnectionState = {
  connection: ConnectionStatus;
  heartbeatWarning: boolean;
  setConnection: (c: ConnectionStatus) => void;
  setHeartbeatWarning: (w: boolean) => void;
};

const DISCONNECTED: ConnectionStatus = { status: 'DISCONNECTED' };

export const useConnection = create<ConnectionState>()(
  devtools(
    (set) => ({
      connection: DISCONNECTED,
      heartbeatWarning: false,
      setConnection: (c) => set({ connection: c }, false, 'setConnection'),
      setHeartbeatWarning: (w) => set({ heartbeatWarning: w }, false, 'setHeartbeatWarning'),
    }),
    { name: 'connection', enabled: import.meta.env.DEV },
  ),
);
