# syntax=docker/dockerfile:1.7

# ============================================================
# Stage 1: Rust builder — WASM package + rpo-api binary
# ============================================================
FROM rust:1.88.0-bookworm AS rust-builder

RUN cargo install wasm-pack

WORKDIR /build

COPY Cargo.toml Cargo.lock rust-toolchain.toml ./
COPY rpo-core rpo-core
COPY rpo-wasm rpo-wasm
COPY rpo-nyx rpo-nyx
COPY rpo-cli rpo-cli
COPY rpo-api rpo-api

# BuildKit cache mounts keep the cargo registry and target dir warm across
# builds. The target dir is ephemeral in the final layer, so the api binary
# is copied into /usr/local/bin where the final COPY --from can find it.
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/build/target \
    wasm-pack build rpo-wasm --target web --release && \
    cargo build --release -p rpo-api && \
    cp target/release/rpo-api /usr/local/bin/rpo-api

# ============================================================
# Stage 2: Frontend builder — Vite production build
#
# The WASM package must land in /build/rpo-wasm/pkg before `npm ci` runs,
# because rpo-app/package.json references it via `file:../rpo-wasm/pkg`.
# WORKDIR /build/rpo-app makes `../rpo-wasm/pkg` resolve identically to
# local dev.
# ============================================================
FROM node:22-bookworm-slim AS frontend-builder

WORKDIR /build/rpo-app

COPY --from=rust-builder /build/rpo-wasm/pkg /build/rpo-wasm/pkg

COPY rpo-app/package.json rpo-app/package-lock.json ./
RUN --mount=type=cache,target=/root/.npm \
    npm ci

COPY rpo-app/ ./
RUN npm run build

# ============================================================
# Stage 3: Runtime
# ============================================================
FROM debian:bookworm-slim AS runtime

RUN apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=rust-builder /usr/local/bin/rpo-api ./rpo-api
COPY --from=frontend-builder /build/rpo-app/dist ./static

# ANISE's MetaAlmanac caches kernels under $HOME via the `directories`
# crate. Point HOME at a volume-mounted dir so the ~50 MB kernel download
# persists across container restarts.
ENV HOME=/data
VOLUME ["/data"]

EXPOSE 3001

CMD ["./rpo-api"]
