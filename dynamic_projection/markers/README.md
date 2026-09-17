# Printable orientation markers

These are exact OpenCV `DICT_6X6_50` markers for ID **1** and ID **0**. Each vector PDF contains two A4 pages: ID 1 first, ID 0 second. The patterns are the same OpenCV-verified cells used for the previous images.

Print `DICT_6X6_50_IDs_1_and_0_80mm.pdf` at **100% / Actual size** for an 80 mm black marker. Retain at least 10 mm white margins on all edges. Check the printed size with a ruler.

`DICT_6X6_50_IDs_1_and_0_24mm.pdf` produces 24 mm black markers at Actual size. Retain at least 3 mm white margins for 30 mm tiles that fit inside the existing 36 mm white chessboard squares. Center ID 0 in the white square above/right of the intended origin and ID 1 below/left to retain the current position defaults. PDF page order is not mounting order.

The 80 mm version cannot fit in those squares. Mount larger markers on the same rigid plane outside the chessboard and projection area, measure each marker center relative to the intended origin, and update the four marker-position flags. Board X points right and Y down on the upright physical print, regardless of image rotation. Divide measured center coordinates in millimeters by 36 to obtain square-length coordinates:

- `--board_origin_marker_x_squares` and `--board_origin_marker_y_squares` for ID 0.
- `--board_secondary_marker_x_squares` and `--board_secondary_marker_y_squares` for ID 1.

Marker size itself is not a calibration-scale input in the current center-based orientation tracker. Resizing is allowed if the codes remain readable, their white margins are preserved, and the center-position flags match the actual mounting locations. Remeasure marker centers relative to the intended origin after mounting, not just marker side lengths. Printed chessboard square size must still be correct. Keep the full chessboard visible during anchoring and re-acquisition. Use matte paper and avoid bending the mounting board.

Regenerate the PDFs with `tools/generate_marker_pdfs.ps1`. It uses exact cell patterns previously generated and verified with OpenCV.
