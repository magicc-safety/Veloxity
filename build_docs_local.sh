#!/bin/bash
set -e

# ==============================================================================
# Script: build_docs_local.sh
# Description: Clean, compile, merge, and build Veloxity documentation system
# ==============================================================================

echo "============================================="
echo "  Starting Veloxity Documentation Build    "
echo "============================================="

# 1. Clear previous build artifacts
echo "Step 1: Cleaning previous build artifacts..."
rm -rf site/
cargo clean --doc

# 2. Compile Rust API docs
echo "Step 2: Compiling Rust API documentation..."

echo "-> Building host-native API docs (sim)..."
cargo doc --no-deps -p sim

echo "-> Building ARM-specific embedded API docs (core, stm_32, nucleo, pixracerpro)..."
cargo doc --no-deps -p veloxity_core -p stm_32 -p nucleo -p pixracerpro --target thumbv7em-none-eabihf

# Ensure virtual environment and zensical are installed using uv
if [ ! -d ".venv" ] || [ ! -f ".venv/bin/zensical" ]; then
    echo "Bootstrapping environment with uv..."
    uv venv .venv
    uv pip install --python .venv zensical
fi

# 3. Build Zensical high-level documentation
echo "Step 3: Building high-level Zensical site..."
.venv/bin/zensical build

# 4. Isolate and merge target doc directories into site/api
echo "Step 4: Merging API documentation into site/api/..."
mkdir -p site/api

# Merge host target docs
if [ -d "target/doc" ]; then
    echo "-> Merging host target API docs..."
    cp -r target/doc/* site/api/
fi

# Merge cross-compiled ARM target docs
if [ -d "target/thumbv7em-none-eabihf/doc" ]; then
    echo "-> Merging ARM target API docs..."
    cp -r target/thumbv7em-none-eabihf/doc/* site/api/
fi

echo "============================================="
echo "  Build Completed Successfully!              "
echo "============================================="
echo ""
echo "To preview the merged documentation locally, run:"
echo "    python3 -m http.server 8000 --directory site"
echo ""
echo "Then open http://localhost:8000 in your browser."
echo "============================================="
