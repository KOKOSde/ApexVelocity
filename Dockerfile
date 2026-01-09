# ApexVelocity Production Dockerfile
# Multi-stage build: C++ core -> Go server -> Python tooling -> slim runtime

# Stage 1: Build C++ core
FROM ubuntu:22.04 AS cpp-builder

RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Root CMake builds the core library into /app/build/core
COPY CMakeLists.txt /app/
COPY core/ /app/core/
COPY config/ /app/config/

RUN mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DAPEXVELOCITY_BUILD_TESTS=OFF && \
    cmake --build . -j"$(nproc)"


# Stage 2: Build Go server
FROM golang:1.25 AS go-builder

WORKDIR /app

COPY server/ /app/server/
COPY config/ /app/config/
COPY --from=cpp-builder /app/build/ /app/build/

ENV CGO_ENABLED=1

RUN cd server && go build -o apex-server ./cmd/apex-server


# Stage 3: Build Python package (optional CLI / tools)
FROM python:3.11-slim AS python-builder

WORKDIR /app

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements-dev.txt /app/
COPY python/ /app/python/
COPY core/ /app/core/

RUN pip install --no-cache-dir -r requirements.txt && \
    pip install --no-cache-dir -e /app/python


# Final runtime image: Go HTTP API + C++ core + (optional) Python tools
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y \
    libyaml-cpp0.7 \
    ca-certificates \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN useradd -m -u 1000 apex
WORKDIR /app

# Copy built artifacts
COPY --from=go-builder /app/server/apex-server /app/
COPY --from=cpp-builder /app/config/ /app/config/
COPY --from=cpp-builder /app/build/core/ /app/build/core/
COPY --from=cpp-builder /app/build/_deps/yaml-cpp-build/ /app/build/_deps/yaml-cpp-build/
COPY --from=python-builder /app/python/ /app/python/

RUN chown -R apex:apex /app
USER apex

EXPOSE 8080

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:8080/health || exit 1

CMD ["./apex-server", "--config", "/app/config"]


