from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Union

import matplotlib.pyplot as plt

DEFAULT_COLOR_PALETTE = [
    "A351FB",
    "FF4040",
    "FFA1A0",
    "FF7633",
    "FFB633",
    "D1D435",
    "4CFB12",
    "94CF1A",
    "40DE8A",
    "1B9640",
    "00D6C1",
    "2E9CAA",
    "00C4FF",
    "364797",
    "6675FF",
    "0019EF",
    "863AFF",
    "530087",
    "CD3AFF",
    "FF97CA",
    "FF39C9",
]

LEGACY_COLOR_PALETTE = [
    "#A351FB",
    "#E6194B",
    "#3CB44B",
    "#FFE119",
    "#0082C8",
    "#F58231",
    "#911EB4",
    "#46F0F0",
    "#F032E6",
    "#D2F53C",
    "#FABEBE",
    "#008080",
    "#E6BEFF",
    "#AA6E28",
    "#FFFAC8",
    "#800000",
    "#AAFFC3",
]

WASTEANT_COLOR_PALETTE = ["C28DFC", "A351FB", "8315F9", "6706CE", "5905B3", "4D049A"]

def _validate_color_hex(color_hex: str):
    color_hex = color_hex.lstrip("#")
    if not all(c in "0123456789abcdefABCDEF" for c in color_hex):
        raise ValueError("Invalid characters in color hash")
    if len(color_hex) not in (3, 6):
        raise ValueError("Invalid length of color hash")
    
@dataclass
class Color:
    """
    Represents a color in RGB format.

    This class provides methods to work with colors, including creating colors from hex
    codes, converting colors to hex strings, RGB tuples, and BGR tuples.

    Attributes:
        r (int): Red channel value (0-255).
        g (int): Green channel value (0-255).
        b (int): Blue channel value (0-255).

    Example:
        ```python
        

        Color.WHITE
        # Color(r=255, g=255, b=255)property
        ```

    | Constant   | Hex Code   | RGB               |
    |------------|------------|-------------------|
    | `WHITE`    | `#FFFFFF`  | `(255, 255, 255)` |
    | `BLACK`    | `#000000`  | `(0, 0, 0)`       |
    | `GREY`     | `#808080`  | `(128, 128, 128)` |
    | `RED`      | `#FF0000`  | `(255, 0, 0)`     |
    | `GREEN`    | `#00FF00`  | `(0, 255, 0)`     |
    | `BLUE`     | `#0000FF`  | `(0, 0, 255)`     |
    | `YELLOW`   | `#FFFF00`  | `(255, 255, 0)`   |
    | `ROBOFLOW` | `#A351FB`  | `(163, 81, 251)`  |
    """

    r: int
    g: int
    b: int

    @classmethod
    def from_hex(cls, color_hex: str) -> Color:
        """
        Create a Color instance from a hex string.

        Args:
            color_hex (str): The hex string representing the color. This string can
                start with '#' followed by either 3 or 6 hexadecimal characters. In
                case of 3 characters, each character is repeated to form the full
                6-character hex code.

        Returns:
            Color: An instance representing the color.

        Example:
            ```python
            # Color(r=255, g=0, b=255)

            Color.from_hex('#f0f')
            # Color(r=255, g=0, b=255)
            ```
        """
        _validate_color_hex(color_hex)
        color_hex = color_hex.lstrip("#")
        if len(color_hex) == 3:
            color_hex = "".join(c * 2 for c in color_hex)
        r, g, b = (int(color_hex[i : i + 2], 16) for i in range(0, 6, 2))
        return cls(r, g, b)

    @classmethod
    def from_rgb_tuple(cls, color_tuple: Tuple[int, int, int]) -> Color:
        """
        Create a Color instance from an RGB tuple.

        Args:
            color_tuple (Tuple[int, int, int]): A tuple representing the color in RGB
                format, where each element is an integer in the range 0-255.

        Returns:
            Color: An instance representing the color.

        Example:
            ```python
            

            Color.from_rgb_tuple((255, 255, 0))
            # Color(r=255, g=255, b=0)
            ```
        """
        r, g, b = color_tuple
        return cls(r=r, g=g, b=b)

    @classmethod
    def from_bgr_tuple(cls, color_tuple: Tuple[int, int, int]) -> Color:
        """
        Create a Color instance from a BGR tuple.

        Args:
            color_tuple (Tuple[int, int, int]): A tuple representing the color in BGR
                format, where each element is an integer in the range 0-255.

        Returns:
            Color: An instance representing the color.

        Example:
            ```python
            

            Color.from_bgr_tuple((0, 255, 255))
            # Color(r=255, g=255, b=0)
            ```
        """
        b, g, r = color_tuple
        return cls(r=r, g=g, b=b)

    def as_hex(self) -> str:
        """
        Converts the Color instance to a hex string.

        Returns:
            str: The hexadecimal color string.

        Example:
            ```python
            

            Color(r=255, g=255, b=0).as_hex()
            # '#ffff00'
            ```
        """
        return f"#{self.r:02x}{self.g:02x}{self.b:02x}"

    def as_rgb(self) -> Tuple[int, int, int]:
        """
        Returns the color as an RGB tuple.

        Returns:
            Tuple[int, int, int]: RGB tuple.

        Example:
            ```python
            

            Color(r=255, g=255, b=0).as_rgb()
            # (255, 255, 0)
            ```
        """
        return self.r, self.g, self.b

    def as_bgr(self) -> Tuple[int, int, int]:
        """
        Returns the color as a BGR tuple.

        Returns:
            Tuple[int, int, int]: BGR tuple.

        Example:
            ```python
            

            Color(r=255, g=255, b=0).as_bgr()
            # (0, 255, 255)
            ```
        """
        return self.b, self.g, self.r

    @property
    def WHITE(cls) -> Color:
        return Color.from_hex("#FFFFFF")

    @property
    def BLACK(cls) -> Color:
        return Color.from_hex("#000000")

    @property
    def GREY(cls) -> Color:
        return Color.from_hex("#808080")

    @property
    def RED(cls) -> Color:
        return Color.from_hex("#FF0000")

    @property
    def GREEN(cls) -> Color:
        return Color.from_hex("#00FF00")

    @property
    def BLUE(cls) -> Color:
        return Color.from_hex("#0000FF")

    @property
    def YELLOW(cls) -> Color:
        return Color.from_hex("#FFFF00")

    @property
    def ROBOFLOW(cls) -> Color:
        return Color.from_hex("#A351FB")

    def __hash__(self):
        return hash((self.r, self.g, self.b))

    def __eq__(self, other):
        return (
            isinstance(other, Color)
            and self.r == other.r
            and self.g == other.g
            and self.b == other.b
        )
    
if __name__ == "__main__":
    color = Color.from_hex("#9b87f5")
    print(color.as_bgr())