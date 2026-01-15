#!/bin/bash
set -e

# Build tracematch for iOS
# Prerequisites:
#   - Rust with iOS targets: rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios
#   - Xcode command line tools
#   - macOS (for xcodebuild)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
OUTPUT_DIR="${PROJECT_DIR}/target/ios"
SWIFT_DIR="${OUTPUT_DIR}/swift"

cd "$PROJECT_DIR"

# Check if libraries are already built (from cache)
LIBS_EXIST=1
[ -f "target/aarch64-apple-ios/release/libtracematch.a" ] || LIBS_EXIST=0
[ -f "target/aarch64-apple-ios-sim/release/libtracematch.a" ] || LIBS_EXIST=0
[ -f "target/x86_64-apple-ios/release/libtracematch.a" ] || LIBS_EXIST=0

# Check if Swift bindings and XCFramework already exist
XCFRAMEWORK_EXISTS=0
[ -d "$OUTPUT_DIR/RouteMatcherFFI.xcframework" ] && XCFRAMEWORK_EXISTS=1
SWIFT_BINDINGS_EXIST=0
[ -f "$SWIFT_DIR/tracematch.swift" ] && SWIFT_BINDINGS_EXISTS=1
[ -f "$SWIFT_DIR/tracematchFFI.h" ] && SWIFT_BINDINGS_EXISTS=1

if [ "$LIBS_EXIST" -eq 1 ] && [ "$XCFRAMEWORK_EXISTS" -eq 1 ] && [ "$SWIFT_BINDINGS_EXISTS" -eq 1 ]; then
  echo "✅ iOS libraries and bindings already built (from cache), skipping build"
  mkdir -p "$OUTPUT_DIR/device" "$OUTPUT_DIR/simulator"
  cp target/aarch64-apple-ios/release/libtracematch.a "$OUTPUT_DIR/device/libtracematch.a"
  lipo -create \
    target/aarch64-apple-ios-sim/release/libtracematch.a \
    target/x86_64-apple-ios/release/libtracematch.a \
    -output "$OUTPUT_DIR/simulator/libtracematch.a" 2>/dev/null || true
  exit 0
fi

if [ "$LIBS_EXIST" -eq 1 ]; then
  echo "✅ iOS libraries found in cache, but bindings need to be generated"
  echo "Skipping Rust compilation (using cached .a files)"
else

echo "============================================"
echo "Building tracematch for iOS"
echo "============================================"
echo ""

# Check prerequisites
echo "Checking prerequisites..."

# Check for required Rust targets
MISSING_TARGETS=""
for TARGET in aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios; do
    if ! rustup target list --installed | grep -q "$TARGET"; then
        MISSING_TARGETS="$MISSING_TARGETS $TARGET"
    fi
done

if [ -n "$MISSING_TARGETS" ]; then
    echo "ERROR: Missing Rust targets:$MISSING_TARGETS"
    echo ""
    echo "Install with:"
    echo "  rustup target add aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios"
    echo ""
    exit 1
fi

echo "All Rust targets installed ✓"
echo ""
fi

# Build for all iOS targets (only if libraries weren't cached)
if [ "$LIBS_EXIST" -eq 0 ]; then
echo "Building for iOS device (aarch64-apple-ios)..."
cargo build --release --target aarch64-apple-ios 

echo ""
echo "Building for iOS simulator - Apple Silicon (aarch64-apple-ios-sim)..."
cargo build --release --target aarch64-apple-ios-sim 

echo ""
echo "Building for iOS simulator - Intel (x86_64-apple-ios)..."
cargo build --release --target x86_64-apple-ios 
fi

# Create output directories
mkdir -p "$OUTPUT_DIR"
mkdir -p "$SWIFT_DIR"

# Create separate directories for device and simulator static libraries
# CocoaPods requires the library to have the SAME NAME in both slices of an XCFramework
DEVICE_LIB_DIR="$OUTPUT_DIR/device"
SIM_LIB_DIR="$OUTPUT_DIR/simulator"
mkdir -p "$DEVICE_LIB_DIR"
mkdir -p "$SIM_LIB_DIR"

# Create universal library for simulator (combines arm64 and x86_64)
echo ""
echo "Creating universal simulator library..."
lipo -create \
    target/aarch64-apple-ios-sim/release/libtracematch.a \
    target/x86_64-apple-ios/release/libtracematch.a \
    -output "$SIM_LIB_DIR/libtracematch.a"

# Copy device library (same name as simulator for XCFramework compatibility)
cp target/aarch64-apple-ios/release/libtracematch.a "$DEVICE_LIB_DIR/libtracematch.a"

# Generate Swift bindings using the embedded uniffi-bindgen
echo ""
echo "Generating Swift bindings..."

# First, build the uniffi-bindgen binary for the host platform
echo "Building uniffi-bindgen..."
if ! cargo build --release --features ffi --bin uniffi-bindgen; then
    echo "ERROR: Failed to build uniffi-bindgen"
    exit 1
fi

# Generate Swift bindings
BINDGEN_SUCCESS=false

if cargo run --release --features ffi --bin uniffi-bindgen generate \
    --library target/aarch64-apple-ios/release/libtracematch.a \
    --language swift \
    --out-dir "$SWIFT_DIR"; then
    BINDGEN_SUCCESS=true
    echo "Swift bindings generated successfully ✓"
else
    # Fallback: try uniffi-bindgen from PATH
    if command -v uniffi-bindgen &> /dev/null; then
        echo "Falling back to system uniffi-bindgen..."
        if uniffi-bindgen generate \
            --library target/aarch64-apple-ios/release/libtracematch.a \
            --language swift \
            --out-dir "$SWIFT_DIR"; then
            BINDGEN_SUCCESS=true
            echo "Swift bindings generated successfully ✓"
        fi
    fi
fi

if [ "$BINDGEN_SUCCESS" = false ]; then
    echo ""
    echo "ERROR: Failed to generate Swift bindings."
    echo ""
    echo "This is required for iOS builds. To fix:"
    echo "  1. Ensure uniffi feature is enabled in Cargo.toml"
    echo "  2. Try: cargo install uniffi_bindgen@0.29"
    echo ""
    exit 1
fi

# Validate generated files
echo ""
echo "Validating generated bindings..."
if [ ! -f "$SWIFT_DIR/tracematch.swift" ]; then
    echo "ERROR: tracematch.swift not generated"
    exit 1
fi
if [ ! -f "$SWIFT_DIR/tracematchFFI.h" ]; then
    echo "ERROR: tracematchFFI.h not generated"
    exit 1
fi
echo "  ✓ tracematch.swift"
echo "  ✓ tracematchFFI.h"

# Generate TypeScript bindings
echo ""
echo "🔧 Generating TypeScript bindings..."
TS_OUTPUT_DIR="../modules/tracematch-native/src/generated"
mkdir -p "$TS_OUTPUT_DIR"

if cargo run --release --features ffi --bin uniffi-bindgen generate \
    --library target/aarch64-apple-ios/release/libtracematch.a \
    --language typescript \
    --out-dir "$TS_OUTPUT_DIR" 2>/dev/null; then
    echo "TypeScript bindings generated successfully ✓"
elif command -v uniffi-bindgen &> /dev/null; then
    echo "Falling back to system uniffi-bindgen..."
    if uniffi-bindgen generate \
        --library target/aarch64-apple-ios/release/libtracematch.a \
        --language typescript \
        --out-dir "$TS_OUTPUT_DIR" 2>/dev/null; then
        echo "TypeScript bindings generated successfully ✓"
    else
        echo "⚠️  TypeScript bindings generation skipped (may need uniffi 0.30+)"
    fi
else
    echo "⚠️  TypeScript bindings generation skipped (may need uniffi 0.30+)"
fi

# Create XCFramework (requires macOS with Xcode)
echo ""
echo "Creating XCFramework..."

# Check if we're on macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    # Create module map for the C header
    HEADERS_DIR="$OUTPUT_DIR/headers"
    mkdir -p "$HEADERS_DIR"

    # Copy the generated header if it exists
    if [ -f "$SWIFT_DIR/tracematchFFI.h" ]; then
        cp "$SWIFT_DIR/tracematchFFI.h" "$HEADERS_DIR/"

        # Create module.modulemap
        # IMPORTANT: The module name must be "tracematchFFI" (lowercase with underscore)
        # to match what UniFFI generates in the Swift bindings:
        #   #if canImport(tracematchFFI)
        #   import tracematchFFI
        cat > "$HEADERS_DIR/module.modulemap" << 'EOF'
module tracematchFFI {
    header "tracematchFFI.h"
    export *
}
EOF
    fi

    rm -rf "$OUTPUT_DIR/RouteMatcherFFI.xcframework"

    # Create XCFramework with headers
    # Use libraries from subdirectories so they have the same name (libtracematch.a)
    # This is required by CocoaPods for vendored XCFrameworks
    if [ -d "$HEADERS_DIR" ] && [ -f "$HEADERS_DIR/tracematchFFI.h" ]; then
        xcodebuild -create-xcframework \
            -library "$DEVICE_LIB_DIR/libtracematch.a" \
            -headers "$HEADERS_DIR" \
            -library "$SIM_LIB_DIR/libtracematch.a" \
            -headers "$HEADERS_DIR" \
            -output "$OUTPUT_DIR/RouteMatcherFFI.xcframework"
        echo "XCFramework created successfully ✓"
    else
        # Create without headers
        xcodebuild -create-xcframework \
            -library "$DEVICE_LIB_DIR/libtracematch.a" \
            -library "$SIM_LIB_DIR/libtracematch.a" \
            -output "$OUTPUT_DIR/RouteMatcherFFI.xcframework"
        echo "XCFramework created (without headers) ✓"
    fi
else
    echo "Skipping XCFramework creation (requires macOS)"
    echo "The static libraries are available at:"
    echo "  - $DEVICE_LIB_DIR/libtracematch.a (device)"
    echo "  - $SIM_LIB_DIR/libtracematch.a (simulator)"
fi

echo ""
echo "============================================"
echo "Build complete!"
echo "============================================"
echo ""
echo "Output directory: $OUTPUT_DIR"
echo ""
echo "Generated files:"
ls -la "$OUTPUT_DIR/" 2>/dev/null || true
echo ""
if [ -d "$SWIFT_DIR" ]; then
    echo "Swift bindings:"
    ls -la "$SWIFT_DIR/" 2>/dev/null || true
fi
echo ""
echo "Next step: Run ./scripts/install-ios.sh to copy files to the native module"
