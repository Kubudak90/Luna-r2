"""tag36h11 bit patterns, laid out exactly like apriltag_to_image() in the AprilRobotics library (apriltag.c):
10×10 cells, an outer ring of white, a ring of black, 6×6 data bits. Cell (row, col) = (y, x) as printed and as
seen from the front. Codes and bit layout copied from tag36h11.c (AprilRobotics/apriltag master)."""

CODES = {0: 0x0000000d7e00984b, 1: 0x0000000dda664ca7, 2: 0x0000000dc4a1c821}
NBITS = 36
TOTAL_WIDTH = 10        # cells, including the white outer ring
BLACK_WIDTH = 8         # cells of the black square: apriltag's tag `size` is this edge
BIT_X = (1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3)
BIT_Y = (1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4)


def bitmap(tag_id=0):
    """10 rows of 10 ints, 1 = white, 0 = black."""
    code = CODES[tag_id]
    im = [[0] * TOTAL_WIDTH for _ in range(TOTAL_WIDTH)]
    for i in range(TOTAL_WIDTH):
        im[0][i] = im[-1][i] = im[i][0] = im[i][-1] = 1
    start = (TOTAL_WIDTH - BLACK_WIDTH) // 2
    for i in range(NBITS):
        if code >> (NBITS - 1 - i) & 1:
            im[BIT_Y[i] + start][BIT_X[i] + start] = 1
    return im
