#!/usr/bin/env python3
"""
Build all targets (firmware + blinky)
Builds each target in its own isolated directory
"""

import sys
import subprocess
from pathlib import Path

# Colors for terminal output
class Colors:
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    RESET = '\033[0m'

def print_colored(message, color=Colors.RESET):
    """Print colored message"""
    print(f"{color}{message}{Colors.RESET}")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Build all targets')
    parser.add_argument('--clean', action='store_true', help='Clean build directories before building')
    args = parser.parse_args()

    print_colored(f"\n{'='*80}", Colors.CYAN)
    print_colored("Building all targets", Colors.CYAN)
    print_colored(f"{'='*80}\n", Colors.CYAN)

    # Build blinky
    print_colored("[1/2] Building blinky...", Colors.YELLOW)
    cmd = [sys.executable, 'build_blinky.py']
    if args.clean:
        cmd.append('--clean')
    
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print_colored("\n[ERROR] Blinky build failed!", Colors.RED)
        return 1

    # Build firmware
    print_colored("\n[2/2] Building firmware...", Colors.YELLOW)
    cmd = [sys.executable, 'build_firmware.py']
    if args.clean:
        cmd.append('--clean')
    
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print_colored("\n[ERROR] Firmware build failed!", Colors.RED)
        return 1

    # Success
    print_colored(f"\n{'='*80}", Colors.GREEN)
    print_colored("[OK] All targets built successfully!", Colors.GREEN)
    print_colored(f"{'='*80}\n", Colors.GREEN)
    
    print_colored("Output directories:", Colors.CYAN)
    print_colored("  - builds/build_blinky/executables/", Colors.RESET)
    print_colored("  - builds/build_firmware/executables/\n", Colors.RESET)

    return 0

if __name__ == '__main__':
    sys.exit(main())
