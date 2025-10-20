import random


def toss_coin():
    """Simulates a single coin toss."""
    return random.choice(["Heads", "Tails"])


def roll_dice():
    """Simulates a six-sided die roll."""
    return random.randint(1, 6)


def get_starting_section():
    """
    Determines the starting section based on two coin tosses.
    Maps coin toss results to the four sections of the track.
    """
    toss1 = toss_coin()
    toss2 = toss_coin()

    if toss1 == "Heads" and toss2 == "Heads":
        section_name = "Top (Heads & Heads)"
    elif toss1 == "Heads" and toss2 == "Tails":
        section_name = "Right (Heads & Tails)"
    elif toss1 == "Tails" and toss2 == "Heads":
        section_name = "Left (Tails & Heads)"
    else:  # Tails & Tails
        section_name = "Bottom (Tails & Tails)"

    return section_name, (toss1, toss2)


def get_corridor_widths():
    """
    Determines the width of the four track corridors.
    Heads = Wide, Tails = Narrow.
    The order is: Starting Section, next clockwise, next, next.
    """
    widths = []
    tosses = []
    for _ in range(4):
        result = toss_coin()
        tosses.append(result)
        widths.append("Wide" if result == "Heads" else "Narrow")
    return widths, tosses


def randomize_open_challenge():
    """Runs the complete randomization for the Open Challenge."""
    print("--- WRO Future Engineers 2025: Open Challenge Randomizer ---")

    # 1. Determine Driving Direction
    direction = random.choice(["Clockwise", "Counter-Clockwise"])
    print(f"\n1. Driving Direction: {direction}")

    # 2. Determine Starting Section
    start_section, start_tosses = get_starting_section()
    print(f"\n2. Starting Section Tosses: {start_tosses[0]}, {start_tosses[1]}")
    print(f"   -> Result: The starting section is '{start_section}'.")

    # 3. Determine Corridor Widths for all four sections
    corridor_widths, width_tosses = get_corridor_widths()
    print("\n3. Corridor Width Tosses (for Starting Section, then clockwise):")
    print(f"   Tosses: {', '.join(width_tosses)}")
    print(f"   -> Corridor widths are: {', '.join(corridor_widths)}")
    print("      - Starting Section Corridor: {}".format(corridor_widths[0]))
    print("      - Next Clockwise Section: {}".format(corridor_widths[1]))
    print("      - Opposite Section: {}".format(corridor_widths[2]))
    print("      - Final Clockwise Section: {}".format(corridor_widths[3]))

    # 4. Determine Exact Starting Zone
    start_zone = roll_dice()
    print(f"\n4. Starting Zone Dice Roll: {start_zone}")
    print(
        f"   -> The robot starts in Zone {start_zone} within the '{start_section}' section."
    )
    print("\n----------------------------------------------------------")


if __name__ == "__main__":
    randomize_open_challenge()
