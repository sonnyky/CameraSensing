# Legacy printable orientation markers

These PDFs contain the previous OpenCV `DICT_6X6_50` markers for IDs **1** and **0**. They are retained as legacy artifacts but are no longer the default orientation setup. The current board uses one `DICT_6X6_50` marker, ID **30**, and selects the nearest outer inner checkerboard corner as the origin.

Print `DICT_6X6_50_IDs_1_and_0_80mm.pdf` at **100% / Actual size** for an 80 mm black marker. Retain at least 10 mm white margins on all edges. Check the printed size with a ruler.

`DICT_6X6_50_IDs_1_and_0_24mm.pdf` produces 24 mm legacy black markers at Actual size. These do **not** fit inside 20 mm checkerboard squares with the required white margin.

For the current single-marker setup, mount ID 30 rigidly outside the checkerboard and projection area, clearly nearer to the intended origin corner than to the other three board corners. Marker size and measured offset are not calibration-scale inputs. Resizing is allowed if the code remains readable and its white margin is preserved. Printed chessboard square size must still be correct. Keep the full chessboard and marker visible together during anchoring and re-acquisition. Use matte paper and avoid bending the mounting board.

Regenerate the PDFs with `tools/generate_marker_pdfs.ps1`. It uses exact cell patterns previously generated and verified with OpenCV.
