#!/usr/bin/env python3
"""
Build script for firmware
Creates a separate build directory with all CMake files isolated
"""

import os
import sys
import shutil
import subprocess
import argparse
from pathlib import Path

# Colors for terminal output
class Colors:
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_colored(message, color=Colors.RESET):
    """Print colored message"""
    print(f"{color}{message}{Colors.RESET}")

def main():
    parser = argparse.ArgumentParser(description='Build firmware')
    parser.add_argument('--clean', action='store_true', help='Clean build directory before building')
    parser.add_argument('--rebuild', action='store_true', help='Clean and rebuild')
    args = parser.parse_args()

    # Configuration
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent.parent
    build_dir = project_root / 'builds' / 'build_firmware'
    target = 'firmware'
    
    print_colored(f"\n{'='*80}", Colors.CYAN)
    print_colored(f"Building {target}", Colors.CYAN)
    print_colored(f"{'='*80}\n", Colors.CYAN)

    # Clean if requested
    if args.clean or args.rebuild:
        if build_dir.exists():
            print_colored(f"Cleaning {build_dir}...", Colors.YELLOW)
            shutil.rmtree(build_dir)
    
    # Create build directory
    build_dir.mkdir(parents=True, exist_ok=True)
    
    # CMake configuration
    cmake_args = [
        'cmake',
        '-G', 'Ninja',
        '../..',  # Source is two levels up from builds/build_firmware
        '-DPLATFORM=stm32',
        '-DCMAKE_SYSTEM_NAME=Generic',
        '-DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY',
        '-DCMAKE_C_COMPILER=arm-none-eabi-gcc',
        '-DCMAKE_CXX_COMPILER=arm-none-eabi-g++',
        '-DCMAKE_OBJCOPY=arm-none-eabi-objcopy',
        '-DCMAKE_BUILD_TYPE=RelWithDebInfo',
        '-DFWTYPE=firmware'
    ]
    
    # Configure
    print_colored("Configuring CMake...", Colors.GREEN)
    try:
        result = subprocess.run(
            cmake_args,
            cwd=build_dir,
            check=True,
            capture_output=False
        )
    except subprocess.CalledProcessError as e:
        print_colored(f"\n[ERROR] CMake configuration failed!", Colors.RED)
        return 1
    
    # Build
    print_colored(f"\nBuilding {target} target...", Colors.GREEN)
    try:
        result = subprocess.run(
            ['cmake', '--build', '.', '--target', target],
            cwd=build_dir,
            check=True,
            capture_output=False
        )
    except subprocess.CalledProcessError as e:
        print_colored(f"\n[ERROR] Build failed!", Colors.RED)
        return 1
    
    # Success
    output_dir = build_dir / 'executables'
    print_colored(f"\n{'='*80}", Colors.GREEN)
    print_colored(f"[OK] Build successful!", Colors.GREEN)
    print_colored(f"{'='*80}", Colors.GREEN)
    print_colored(f"\nOutput files in: {output_dir}/", Colors.CYAN)
    print_colored(f"  - {target}", Colors.RESET)
    print_colored(f"  - {target}.bin", Colors.RESET)
    print_colored(f"  - {target}.hex", Colors.RESET)
    print_colored(f"  - {target}.map (in build root)\n", Colors.RESET)
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
