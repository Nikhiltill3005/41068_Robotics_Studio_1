#!/usr/bin/env python3
"""
Randomize Fire Positions in SDF World File

This script automatically randomizes the x,y positions of fire entities
in the bushland_spaced.sdf world file. It completely replaces the fire
section with the specified number of fires at random positions.

Usage:
    python3 randomize_fires.py

Configuration:
    NUM_FIRES: Change this to set how many fires you want (default: 5)
    MIN_RADIUS: Minimum distance from origin (default: 3m, avoids charging station)
    MAX_RADIUS: Maximum distance from origin (default: 23m, stays within bounds)
"""

import random
import re
import os
import sys

# ===== CONFIGURATION =====
NUM_FIRES = 12       # <<< CHANGE THIS to set how many fires you want
MIN_RADIUS = 3      # Avoid center area (charging station)
MAX_RADIUS = 23     # Keep within bounds
# =========================

WORLD_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '../worlds/bushland_spaced.sdf'
)

def generate_random_position(min_radius=MIN_RADIUS, max_radius=MAX_RADIUS, avoid_center=True):
    """Generate random position avoiding center area"""
    while True:
        x = random.uniform(-max_radius, max_radius)
        y = random.uniform(-max_radius, max_radius)

        # Avoid center area if specified (charging station is at origin)
        if avoid_center and (abs(x) < min_radius and abs(y) < min_radius):
            continue

        return x, y

def generate_fire_xml(fire_number, x, y):
    """Generate XML for a single fire entity"""
    # Use different rotation for variety
    rotation = random.uniform(0, 6.28)  # 0 to 2*pi radians

    if fire_number == 1:
        name = "test_fire"
    else:
        name = f"test_fire{fire_number}"

    return f"""    <include>
      <uri>model://fire</uri>
      <name>{name}</name>
      <pose>{x:.1f} {y:.1f} 0 0 0.05 {rotation:.2f}</pose>
      <static>true</static>
    </include>"""

def replace_fire_section(sdf_content, num_fires):
    """
    Replace the entire fire section with new randomly positioned fires.

    Finds the fire section (between comment markers) and replaces it
    with the specified number of fires at random positions.
    """

    # Pattern to match the entire fire section
    # Matches from "ONLY X FIRES" comment to the end of last fire block
    fire_section_pattern = re.compile(
        r'(\s*<!-- ONLY \d+ FIRES.*?-->.*?)'  # Comment header
        r'(<include>.*?<uri>model://fire</uri>.*?</include>\s*)+'  # All fire blocks
        r'(\n)',
        re.MULTILINE | re.DOTALL
    )

    match = fire_section_pattern.search(sdf_content)

    if not match:
        print("ERROR: Could not find fire section in SDF file!")
        print("Looking for pattern: '<!-- ONLY X FIRES' followed by fire includes")
        return sdf_content

    # Generate random positions for all fires
    print(f"\nGenerating {num_fires} fire(s) at random positions:")
    fire_blocks = []
    for i in range(num_fires):
        x, y = generate_random_position()
        fire_xml = generate_fire_xml(i + 1, x, y)
        fire_blocks.append(fire_xml)
        print(f"  Fire {i+1}: ({x:.1f}, {y:.1f})")

    # Build the new fire section
    new_fire_section = f"""    <!-- ONLY {num_fires} FIRES - Randomly dispersed across the map -->
"""
    new_fire_section += "\n\n".join(fire_blocks)
    new_fire_section += "\n"

    # Replace the old fire section with the new one
    start = match.start()
    end = match.end()
    result = sdf_content[:start] + new_fire_section + sdf_content[end:]

    return result

def main():
    """Main function to randomize fires in world file"""

    print("=" * 60)
    print("Fire Position Randomizer for bushland_spaced.sdf")
    print("=" * 60)
    print(f"\nConfiguration:")
    print(f"  Number of fires: {NUM_FIRES}")
    print(f"  Min radius: {MIN_RADIUS}m (avoid charging station)")
    print(f"  Max radius: {MAX_RADIUS}m (stay within bounds)")

    # Check if world file exists
    if not os.path.exists(WORLD_FILE):
        print(f"\nERROR: World file not found: {WORLD_FILE}")
        sys.exit(1)

    print(f"\nReading world file: {WORLD_FILE}")

    # Read the SDF file
    with open(WORLD_FILE, 'r') as f:
        sdf_content = f.read()

    # Replace fire section with new fires
    print(f"\nReplacing fire section with {NUM_FIRES} random fires...")
    updated_content = replace_fire_section(sdf_content, NUM_FIRES)

    # Write back to file
    print(f"\nWriting updated world file...")
    with open(WORLD_FILE, 'w') as f:
        f.write(updated_content)

    print(f"\n✓ Fire section replaced successfully!")
    print(f"✓ World file updated: {WORLD_FILE}")
    print(f"\nYou can now launch the simulation with {NUM_FIRES} randomized fires.")
    print("\nTo change the number of fires, edit NUM_FIRES at the top of this script.")
    print("=" * 60)

if __name__ == "__main__":
    main()
