# syntax=docker/dockerfile:1

# Stage 1: Build WASM
FROM rust:1.95-bookworm AS wasm-builder

RUN curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh

WORKDIR /build

# Cache Cargo registry and git deps across builds
COPY Cargo.toml Cargo.lock ./
COPY tracematch-wasm/Cargo.toml tracematch-wasm/Cargo.toml
RUN mkdir -p src && echo "" > src/lib.rs && mkdir -p tracematch-wasm/src && echo "fn main(){}" > tracematch-wasm/src/lib.rs \
    && cargo fetch

# Copy real source and build
COPY src/ src/
COPY tracematch-wasm/src/ tracematch-wasm/src/
RUN --mount=type=cache,target=/build/target \
    --mount=type=cache,target=/usr/local/cargo/registry \
    cd tracematch-wasm && wasm-pack build --target web --release \
    && cp -r pkg /build/wasm-pkg

# Stage 1b: Dev WASM watcher (reuses wasm-builder toolchain)
FROM wasm-builder AS wasm-dev
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    cargo install cargo-watch

# Stage 2: Build web frontend
FROM node:22-slim AS web-builder

WORKDIR /build/web
COPY web/package.json web/package-lock.json ./
RUN npm ci

COPY web/ .
COPY --from=wasm-builder /build/wasm-pkg/ src/lib/wasm/pkg/
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
