# syntax=docker/dockerfile:1

# ── Dev stage: tools only, no source ──────────────────────────────
# Used by docker-compose wasm service. Source is bind-mounted at
# runtime, so this image only rebuilds when tools change.
FROM rust:1.95-bookworm AS wasm-dev

RUN curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    cargo install cargo-watch

WORKDIR /build

# ── Production WASM build ─────────────────────────────────────────
FROM rust:1.95-bookworm AS wasm-builder

RUN curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh

WORKDIR /build
COPY . .
RUN --mount=type=cache,target=/build/target \
    --mount=type=cache,target=/usr/local/cargo/registry \
    cd tracematch-wasm && wasm-pack build --target web --release \
    && cp -r pkg /build/wasm-pkg

# ── Production web frontend ──────────────────────────────────────
FROM node:22-slim AS web-builder

WORKDIR /build/web
COPY web/package.json web/package-lock.json ./
RUN npm ci

COPY web/ .
COPY --from=wasm-builder /build/wasm-pkg/ src/lib/wasm/pkg/
RUN npm run build

# ── Production serve ─────────────────────────────────────────────
FROM nginx:alpine

COPY --from=web-builder /build/web/build/ /usr/share/nginx/html/
COPY <<'EOF' /etc/nginx/conf.d/default.conf
server {
    listen 8080;
    root /usr/share/nginx/html;
    index index.html;

    location / {
        try_files $uri $uri/ /index.html;
    }

    location /wasm/ {
        types {
            application/wasm wasm;
            application/javascript js;
        }
    }

    gzip on;
    gzip_types text/html text/css application/javascript application/wasm application/json;
}
EOF

EXPOSE 8080
