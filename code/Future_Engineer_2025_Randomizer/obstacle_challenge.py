import os
import random
import webbrowser

from PIL import Image, ImageDraw, ImageFont

# --- CONFIGURATION & CONSTANTS ---

# NOTE: This script requires the Pillow library.
# You can install it by running: pip install Pillow
# It also expects 'WRO-2025_FutureEngineers_Playfield.jpg' to be in the same directory.

# Set to True to rotate the final image so the parking lot is always at the bottom.
ROTATE_TO_START = True

BASE_IMAGE_PATH = "WRO-2025_FutureEngineers_Playfield.jpg"
OUTPUT_IMAGE_PATH = "WRO_Obstacle_Challenge_Layout.png"
PILLAR_RADIUS = 25
PARKING_WIDTH = 160
PARKING_LENGTH = 100
FONT_SIZE = 60

# Colors for drawing
COLORS = {
    "Red": "#d90429",
    "Green": "#008000",
    "Parking": "#F702F9",
    "Text": "#000000",
    "Arrow": "#000000",
}

# --- COORDINATES (based on a 2134x2134 image) ---

# Positions for pillars within each straight section
# 'p1' is first in clockwise order, 'p2' is middle, 'p3' is last.
SECTION_PILLAR_COORDS_OUTWARD = {
    "Top": {"p1": (710, 320), "p2": (1025, 320), "p3": (1345, 320)},
    "Right": {"p1": (1725, 710), "p2": (1725, 1025), "p3": (1725, 1345)},
    "Bottom": {"p1": (1345, 1725), "p2": (1025, 1725), "p3": (710, 1725)},
    "Left": {"p1": (320, 1345), "p2": (320, 1025), "p3": (320, 710)},
}
# Coords for when pillars are moved inward
SECTION_PILLAR_COORDS_INWARD = {
    "Top": {"p1": (710, 450), "p2": (1025, 450), "p3": (1345, 450)},
    "Right": {"p1": (1600, 710), "p2": (1600, 1025), "p3": (1600, 1345)},
    "Bottom": {"p1": (1345, 1600), "p2": (1025, 1600), "p3": (710, 1600)},
    "Left": {"p1": (450, 1345), "p2": (450, 1025), "p3": (450, 710)},
}

# Coordinates for drawing the two parallel lines of the parking lot
PARKING_LOT_COORDS = {
    "Top": [
        ((710, 70), (710, 70 + PARKING_LENGTH)),
        ((710 + PARKING_WIDTH, 70), (710 + PARKING_WIDTH, 70 + PARKING_LENGTH)),
    ],
    "Right": [
        ((1975, 710), (1975 - PARKING_LENGTH, 710)),
        ((1975, 710 + PARKING_WIDTH), (1975 - PARKING_LENGTH, 710 + PARKING_WIDTH)),
    ],
    "Bottom": [
        ((1340, 1975), (1340, 1975 - PARKING_LENGTH)),
        ((1340 - PARKING_WIDTH, 1975), (1340 - PARKING_WIDTH, 1975 - PARKING_LENGTH)),
    ],
    "Left": [
        ((70, 1340), (70 + PARKING_LENGTH, 1340)),
        ((70, 1340 - PARKING_WIDTH), (70 + PARKING_LENGTH, 1340 - PARKING_WIDTH)),
    ],
}

# Coordinates for direction arrows on the track
ARROW_COORDS = {
    "Clockwise": [
        {
            "line": ((850, 385), (1200, 385)),
            "head": [(1200, 365), (1240, 385), (1200, 405)],
        },  # Top
        {
            "line": ((1665, 850), (1665, 1200)),
            "head": [(1645, 1200), (1665, 1240), (1685, 1200)],
        },  # Right
        {
            "line": ((1200, 1665), (850, 1665)),
            "head": [(850, 1645), (810, 1665), (850, 1685)],
        },  # Bottom
        {
            "line": ((385, 1200), (385, 850)),
            "head": [(365, 850), (385, 810), (405, 850)],
        },  # Left
    ],
    "Counter-Clockwise": [
        {
            "line": ((1200, 385), (850, 385)),
            "head": [(850, 365), (810, 385), (850, 405)],
        },  # Top
        {
            "line": ((1665, 1200), (1665, 850)),
            "head": [(1645, 850), (1665, 810), (1685, 850)],
        },  # Right
        {
            "line": ((850, 1665), (1200, 1665)),
            "head": [(1200, 1645), (1240, 1665), (1200, 1685)],
        },  # Bottom
        {
            "line": ((385, 850), (385, 1200)),
            "head": [(365, 1200), (385, 1240), (405, 1200)],
        },  # Left
    ],
}

# Data structure mapping card number to pillar configurations.
# Each pillar is a tuple of (position, color, row).
CARD_LAYOUTS = {
    1: {"pillars": [("p3", "Green", "Inner")]},
    2: {"pillars": [("p3", "Red", "Inner")]},
    3: {"pillars": [("p2", "Green", "Inner")]},
    4: {"pillars": [("p2", "Red", "Inner")]},
    5: {"pillars": [("p1", "Green", "Inner")]},
    6: {"pillars": [("p1", "Red", "Inner")]},
    7: {"pillars": [("p3", "Green", "Outer")]},
    8: {"pillars": [("p3", "Red", "Outer")]},
    9: {"pillars": [("p2", "Green", "Outer")]},
    10: {"pillars": [("p2", "Red", "Outer")]},
    11: {"pillars": [("p1", "Green", "Inner")]},
    12: {"pillars": [("p1", "Red", "Inner")]},
    13: {"pillars": [("p3", "Green", "Outer"), ("p1", "Green", "Inner")]},
    14: {"pillars": [("p3", "Green", "Outer"), ("p1", "Red", "Inner")]},
    15: {"pillars": [("p3", "Red", "Outer"), ("p1", "Green", "Inner")]},
    16: {"pillars": [("p3", "Green", "Outer"), ("p1", "Red", "Inner")]},
    17: {"pillars": [("p3", "Red", "Outer"), ("p1", "Green", "Inner")]},
    18: {"pillars": [("p3", "Red", "Outer"), ("p1", "Red", "Inner")]},
    19: {"pillars": [("p3", "Green", "Inner"), ("p1", "Green", "Outer")]},
    20: {"pillars": [("p3", "Green", "Inner"), ("p1", "Red", "Outer")]},
    21: {"pillars": [("p3", "Red", "Inner"), ("p1", "Green", "Outer")]},
    22: {"pillars": [("p3", "Green", "Inner"), ("p1", "Red", "Outer")]},
    23: {"pillars": [("p3", "Red", "Inner"), ("p1", "Green", "Outer")]},
    24: {"pillars": [("p3", "Red", "Inner"), ("p1", "Red", "Outer")]},
    25: {"pillars": [("p3", "Green", "Inner"), ("p1", "Green", "Inner")]},
    26: {"pillars": [("p3", "Green", "Inner"), ("p1", "Red", "Inner")]},
    27: {"pillars": [("p3", "Red", "Inner"), ("p1", "Green", "Inner")]},
    28: {"pillars": [("p3", "Green", "Inner"), ("p1", "Red", "Inner")]},
    29: {"pillars": [("p3", "Red", "Inner"), ("p1", "Green", "Inner")]},
    30: {"pillars": [("p3", "Red", "Inner"), ("p1", "Red", "Inner")]},
    31: {"pillars": [("p3", "Green", "Outer"), ("p1", "Green", "Outer")]},
    32: {"pillars": [("p3", "Green", "Outer"), ("p1", "Red", "Outer")]},
    33: {"pillars": [("p3", "Red", "Outer"), ("p1", "Green", "Outer")]},
    34: {"pillars": [("p3", "Green", "Outer"), ("p1", "Red", "Outer")]},
    35: {"pillars": [("p3", "Red", "Outer"), ("p1", "Green", "Outer")]},
    36: {"pillars": [("p3", "Red", "Outer"), ("p1", "Red", "Outer")]},
}


# --- HELPER FUNCTIONS ---


def toss_coin():
    """Simulates a single coin toss."""
    return random.choice(["Heads", "Tails"])


def get_section_from_tosses():
    """Determines a track section based on two coin tosses."""
    toss1 = toss_coin()
    toss2 = toss_coin()

    if toss1 == "Heads" and toss2 == "Heads":
        section_name = "Top"
        full_name = "Top (Heads & Heads)"
    elif toss1 == "Heads" and toss2 == "Tails":
        section_name = "Right"
        full_name = "Right (Heads & Tails)"
    elif toss1 == "Tails" and toss2 == "Heads":
        section_name = "Left"
        full_name = "Left (Tails & Heads)"
    else:  # Tails & Tails
        section_name = "Bottom"
        full_name = "Bottom (Tails & Tails)"

    return section_name, full_name, (toss1, toss2)


# --- DRAWING FUNCTIONS ---


def draw_pillar(draw, center_xy, color):
    """Draws a colored circle (pillar) on the image."""
    x, y = center_xy
    bbox = [x - PILLAR_RADIUS, y - PILLAR_RADIUS, x + PILLAR_RADIUS, y + PILLAR_RADIUS]
    draw.ellipse(bbox, fill=COLORS[color], outline="black", width=2)


def draw_parking_lot(draw, section):
    """Draws the parking lot lines for a given section."""
    coords = PARKING_LOT_COORDS[section]
    draw.line(coords[0], fill=COLORS["Parking"], width=10)
    draw.line(coords[1], fill=COLORS["Parking"], width=10)


def draw_direction_arrows(draw, direction):
    """Draws arrows on the track to indicate driving direction."""
    arrows = ARROW_COORDS[direction]
    for arrow in arrows:
        draw.line(arrow["line"], fill=COLORS["Arrow"], width=15)
        draw.polygon(arrow["head"], fill=COLORS["Arrow"])


def draw_configuration(config):
    """Loads the base image and draws the randomized configuration on it."""
    try:
        base_image = Image.open(BASE_IMAGE_PATH).convert("RGBA")
    except FileNotFoundError:
        print(f"Error: Base image '{BASE_IMAGE_PATH}' not found.")
        print(
            "Please make sure the playfield image is in the same directory as the script."
        )
        return

    draw = ImageDraw.Draw(base_image)

    # Draw Direction Arrows
    draw_direction_arrows(draw, config["direction"])

    # Draw Parking Lot
    draw_parking_lot(draw, config["parking_section"])

    # Combine the single sign section and other sections for drawing
    all_sections_layout = config["other_sections_layout"].copy()
    all_sections_layout[config["single_sign_section"]] = config["single_sign_card"]

    # Draw all pillars from cards
    for section, card_num in all_sections_layout.items():
        layout = CARD_LAYOUTS[card_num]

        for pillar_data in layout["pillars"]:
            pillar_pos, pillar_color, pillar_row = pillar_data

            # Determine if the pillar is inward or outward
            # It's forced inward if the parking lot is in the section.
            # Otherwise, its position is determined by the card.
            if section == config["parking_section"] or pillar_row == "Inner":
                is_inward = True
            else:
                is_inward = False

            pillar_coords = (
                SECTION_PILLAR_COORDS_INWARD
                if is_inward
                else SECTION_PILLAR_COORDS_OUTWARD
            )
            center_xy = pillar_coords[section][pillar_pos]
            draw_pillar(draw, center_xy, pillar_color)

    # --- Optional Rotation ---
    # If enabled, rotate the image so the parking section is at the bottom.
    if ROTATE_TO_START:
        parking_section = config["parking_section"]
        rotation_angle = 0
        if parking_section == "Right":
            rotation_angle = 270
        elif parking_section == "Top":
            rotation_angle = 180
        elif parking_section == "Left":
            rotation_angle = 90

        if rotation_angle != 0:
            # The rotate method rotates counter-clockwise
            base_image = base_image.rotate(rotation_angle, expand=True)
    # --- End of Optional Rotation ---

    # Save and show the final image
    base_image.save(OUTPUT_IMAGE_PATH)
    print(f"\nSuccessfully generated layout image: '{OUTPUT_IMAGE_PATH}'")
    webbrowser.open("file://" + os.path.realpath(OUTPUT_IMAGE_PATH))


# --- MAIN LOGIC ---


def randomize_obstacle_challenge():
    """Runs the complete randomization and generates a visual layout."""
    print("--- WRO Future Engineers 2025: Obstacle Challenge Randomizer ---")

    config = {}

    # 1. Determine Driving Direction
    config["direction"] = random.choice(["Clockwise", "Counter-Clockwise"])
    print(f"\n1. Driving Direction: {config['direction']}")

    # 2. Determine Section with a Single Traffic Sign
    s_name, s_full_name, s_tosses = get_section_from_tosses()
    config["single_sign_section"] = s_name
    print(f"\n2. Single Sign Section Tosses: {s_tosses[0]}, {s_tosses[1]}")
    print(f"   -> Result: The section with one traffic sign is '{s_full_name}'.")

    # 3. Determine the Color of the Single Traffic Sign (which determines its card)
    color_toss = toss_coin()
    if color_toss == "Heads":
        config["single_sign_card"] = 9  # Green
        print(f"\n3. Single Sign Color Toss: {color_toss} -> Green (Card #9)")
    else:
        config["single_sign_card"] = 10  # Red
        print(f"\n3. Single Sign Color Toss: {color_toss} -> Red (Card #10)")

    # 4. Determine Traffic Sign Layouts for the other three sections
    print("\n4. Traffic Sign Layouts for Other Sections:")
    cards = list(range(1, 37))

    # Remove the card used for the single sign
    cards.remove(config["single_sign_card"])
    print(
        f"   - Card #{config['single_sign_card']} was used for the single sign and is removed from the deck."
    )

    random.shuffle(cards)

    sections = ["Top", "Right", "Bottom", "Left"]
    other_sections = [s for s in sections if s != config["single_sign_section"]]

    config["other_sections_layout"] = {
        other_sections[0]: cards.pop(),
        other_sections[1]: cards.pop(),
        other_sections[2]: cards.pop(),
    }

    print(
        f"   -> Section '{other_sections[0]}' gets Card #{config['other_sections_layout'][other_sections[0]]}"
    )
    print(
        f"   -> Section '{other_sections[1]}' gets Card #{config['other_sections_layout'][other_sections[1]]}"
    )
    print(
        f"   -> Section '{other_sections[2]}' gets Card #{config['other_sections_layout'][other_sections[2]]}"
    )

    # 5. Determine the Parking Lot / Starting Section
    p_name, p_full_name, p_tosses = get_section_from_tosses()
    config["parking_section"] = p_name
    print(f"\n5. Parking Lot Section Tosses: {p_tosses[0]}, {p_tosses[1]}")
    print(f"   -> Result: The Parking Lot is in the '{p_full_name}' section.")
    print("\n----------------------------------------------------------")

    # Generate and display the image
    draw_configuration(config)


if __name__ == "__main__":
    randomize_obstacle_challenge()
