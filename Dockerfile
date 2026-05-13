# Stage 1: Build WASM
FROM rust:1.95-bookworm AS wasm-builder

# wasm-bindgen-rayon requires nightly Rust for `-Z build-std` to enable
# WASM threading (atomics + bulk-memory). The host page must also ship
# COOP/COEP headers for SharedArrayBuffer support — see vite.config.ts
# (dev) and the production deploy notes.
RUN rustup toolchain install nightly-2025-09-15 --component rust-src \
    && curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh

WORKDIR /build
COPY . .
RUN cd tracematch-wasm && \
    RUSTUP_TOOLCHAIN=nightly-2025-09-15 \
    RUSTFLAGS="-C target-feature=+atomics,+bulk-memory,+mutable-globals" \
    wasm-pack build --target web --release --features parallel -- -Z build-std=std,panic_abort

# Stage 2: Build web frontend
FROM node:22-slim AS web-builder

WORKDIR /build/web
COPY web/package.json web/package-lock.json ./
RUN npm ci

COPY web/ .
COPY --from=wasm-builder /build/tracematch-wasm/pkg/ static/wasm/
RUN npm run build

# Stage 3: Serve
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
