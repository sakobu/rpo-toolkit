import { err, match, ok, type Result } from '@railway-ts/pipelines/result';
import { validate } from '@railway-ts/pipelines/schema';

import {
  type CancelMsg,
  type ClientMessage,
  type ResponseForRequest,
  type ServerError,
  type ServerMessage,
  serverMessageSchema,
} from '@/schemas/wsProtocol';
import { useConnection } from '@/stores/connection';

// ─── Tunable values ────────────────────────────────────────────────────────

const HEARTBEAT_WARNING_MS = 60_000;
const HEARTBEAT_CHECK_INTERVAL_MS = 5_000;
const BACKOFF_BASE_MS = 250;
const BACKOFF_MAX_MS = 8_000;
const JITTER = 0.25;

function resolveWsUrl(): string {
  const explicit = import.meta.env.VITE_WS_URL;
  if (explicit && explicit.length > 0) return explicit;
  // Dev fallback: vite proxies /ws to ws://localhost:3001 (see vite.config.ts).
  const { protocol, host } = window.location;
  const wsProto = protocol === 'https:' ? 'wss:' : 'ws:';
  return `${wsProto}//${host}/ws`;
}

type RequestKind = Exclude<ClientMessage, CancelMsg>;

// Storage erases the typed response — `send<TReq>` re-narrows at its own
// Promise boundary (see the wrapper resolver in `send`). Keeping Pending
// non-generic means dispatch can call `pending.resolve(ok(msg))` directly
// without a second cast.
type Pending = {
  resolve: (r: Result<unknown, ServerError>) => void;
};

// ─── Module-level singleton state ──────────────────────────────────────────

let socket: WebSocket | null = null;
let shouldReconnect = false;
let reconnectAttempt = 0;
let reconnectTimer: number | null = null;
let heartbeatTimer: number | null = null;
let lastHeartbeatAt = Date.now();
const inFlight = new Map<number, Pending>();

// ─── Public API ────────────────────────────────────────────────────────────

export function connect(): void {
  if (
    socket &&
    (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)
  ) {
    return;
  }
  shouldReconnect = true;
  openSocket();
  startHeartbeatWatchdog();
}

export function disconnect(): void {
  shouldReconnect = false;
  if (reconnectTimer !== null) {
    window.clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  stopHeartbeatWatchdog();
  drainInFlight({
    code: 'connection_lost',
    message: 'WebSocket disconnected',
  });
  if (socket) {
    socket.onopen = null;
    socket.onclose = null;
    socket.onerror = null;
    socket.onmessage = null;
    try {
      socket.close();
    } catch {
      // no-op: already closed
    }
    socket = null;
  }
  useConnection.getState().setConnection({ status: 'DISCONNECTED' });
  useConnection.getState().setHeartbeatWarning(false);
}

export function send<TReq extends RequestKind>(
  msg: TReq,
): Promise<Result<ResponseForRequest<TReq>, ServerError>> {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    return Promise.resolve(
      err({
        code: 'connection_lost',
        message: 'WebSocket is not open',
      } satisfies ServerError),
    );
  }
  return new Promise<Result<ResponseForRequest<TReq>, ServerError>>((resolve) => {
    // Wrap the typed resolver so storage can stay `Pending` (unknown value).
    // The narrow from `Result<unknown, ...>` to the call-site's response type
    // is valid because dispatch keys off `msg.request_id`, which only this
    // send() produced.
    inFlight.set(msg.request_id, {
      resolve: (r) => resolve(r as Result<ResponseForRequest<TReq>, ServerError>),
    });
    try {
      socket!.send(JSON.stringify(msg));
      updateBusyCount();
    } catch (e) {
      inFlight.delete(msg.request_id);
      resolve(
        err({
          code: 'connection_lost',
          message: `send failed: ${String(e)}`,
        } satisfies ServerError),
      );
    }
  });
}

export function cancel(requestId: number): void {
  if (!socket || socket.readyState !== WebSocket.OPEN) return;
  const msg: CancelMsg = { type: 'cancel', request_id: requestId };
  try {
    socket.send(JSON.stringify(msg));
  } catch {
    // no-op: cancellation is best-effort
  }
}

// ─── Lifecycle internals ───────────────────────────────────────────────────

function openSocket(): void {
  useConnection.getState().setConnection({ status: 'CONNECTING' });
  const s = new WebSocket(resolveWsUrl());
  socket = s;

  s.onopen = () => {
    reconnectAttempt = 0;
    lastHeartbeatAt = Date.now();
    const store = useConnection.getState();
    store.setHeartbeatWarning(false);
    // `updateBusyCount` alone won't flip CONNECTING/RECONNECTING → IDLE (it
    // early-returns on those so BUSY/IDLE writes don't stomp transitional
    // states). Set IDLE directly here; any in-flight entries then promote to
    // BUSY on the next send.
    store.setConnection(
      inFlight.size === 0 ? { status: 'IDLE' } : { status: 'BUSY', inFlightCount: inFlight.size },
    );
  };

  s.onmessage = (event) => {
    if (typeof event.data !== 'string') {
      // Binary frames are rejected server-side; ignore defensively.
      return;
    }
    handleFrame(event.data);
  };

  s.onerror = () => {
    // The subsequent onclose will drive state; nothing else to do here.
  };

  s.onclose = () => {
    // If this is the socket we still own, drain + schedule reconnect. If a
    // fresh connect() replaced it, leave it alone.
    if (socket !== s) return;
    socket = null;
    drainInFlight({
      code: 'connection_lost',
      message: 'WebSocket closed',
    });
    if (!shouldReconnect) {
      useConnection.getState().setConnection({ status: 'DISCONNECTED' });
      return;
    }
    scheduleReconnect();
  };
}

function scheduleReconnect(): void {
  const delay = backoffDelay(reconnectAttempt);
  useConnection.getState().setConnection({ status: 'RECONNECTING', attempt: reconnectAttempt + 1 });
  reconnectTimer = window.setTimeout(() => {
    reconnectTimer = null;
    reconnectAttempt += 1;
    if (!shouldReconnect) return;
    openSocket();
  }, delay);
}

function backoffDelay(attempt: number): number {
  const base = Math.min(BACKOFF_BASE_MS * 2 ** attempt, BACKOFF_MAX_MS);
  const jitter = base * JITTER * (Math.random() * 2 - 1);
  return Math.max(BACKOFF_BASE_MS, base + jitter);
}

// ─── Frame handling ────────────────────────────────────────────────────────

function handleFrame(raw: string): void {
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch {
    // Malformed JSON from the server is a contract violation; ignore frame.
    return;
  }
  // `validate` confirms the discriminator + shape at runtime; the narrow to
  // `ServerMessage` is needed because `discriminatedUnion` widens `type` back
  // to `string` in its output type, and wsProtocol.ts passes rpo-wasm payloads
  // through as `unknown` (see the comment there for why). Err payload is
  // unused — the dev-mode warn only reads `parsed`.
  const result = validate(parsed, serverMessageSchema) as Result<ServerMessage, unknown>;
  match(result, {
    ok: dispatch,
    err: () => {
      if (import.meta.env.DEV) {
        console.warn('[ws] invalid server frame', parsed);
      }
    },
  });
}

function dispatch(msg: ServerMessage): void {
  switch (msg.type) {
    case 'heartbeat': {
      lastHeartbeatAt = Date.now();
      useConnection.getState().setHeartbeatWarning(false);
      return;
    }
    case 'progress': {
      // Phase 2 has no UI for progress; schema is validated so Phase 5+ can wire.
      return;
    }
    case 'error': {
      if (msg.request_id === undefined) return;
      const pending = inFlight.get(msg.request_id);
      if (!pending) return;
      inFlight.delete(msg.request_id);
      pending.resolve(
        err({
          request_id: msg.request_id,
          code: msg.code,
          message: msg.message,
          detail: msg.detail,
        }),
      );
      updateBusyCount();
      return;
    }
    case 'drag_result':
    case 'validation_result':
    case 'monte_carlo_result':
    case 'cancelled': {
      const pending = inFlight.get(msg.request_id);
      if (!pending) return;
      inFlight.delete(msg.request_id);
      pending.resolve(ok(msg));
      updateBusyCount();
      return;
    }
  }
}

function drainInFlight(error: ServerError): void {
  for (const pending of inFlight.values()) {
    pending.resolve(err(error));
  }
  inFlight.clear();
  updateBusyCount();
}

function updateBusyCount(): void {
  const store = useConnection.getState();
  const current = store.connection;
  if (
    current.status === 'DISCONNECTED' ||
    current.status === 'CONNECTING' ||
    current.status === 'RECONNECTING'
  ) {
    return;
  }
  if (inFlight.size === 0) {
    if (current.status !== 'IDLE') store.setConnection({ status: 'IDLE' });
  } else if (current.status !== 'BUSY' || current.inFlightCount !== inFlight.size) {
    // Guard against rewriting an identical BUSY shape — zustand has no
    // structural dedupe, so a new object literal would re-notify subscribers.
    store.setConnection({ status: 'BUSY', inFlightCount: inFlight.size });
  }
}

// ─── Heartbeat watchdog ────────────────────────────────────────────────────

function startHeartbeatWatchdog(): void {
  if (heartbeatTimer !== null) return;
  heartbeatTimer = window.setInterval(() => {
    if (inFlight.size === 0) return;
    const stale = Date.now() - lastHeartbeatAt > HEARTBEAT_WARNING_MS;
    const { heartbeatWarning, setHeartbeatWarning } = useConnection.getState();
    if (stale !== heartbeatWarning) setHeartbeatWarning(stale);
  }, HEARTBEAT_CHECK_INTERVAL_MS);
}

function stopHeartbeatWatchdog(): void {
  if (heartbeatTimer !== null) {
    window.clearInterval(heartbeatTimer);
    heartbeatTimer = null;
  }
}
