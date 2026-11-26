param([string]$OutputDirectory = (Join-Path $PSScriptRoot '..\markers'))

$ErrorActionPreference = 'Stop'
# Exact DICT_6X6_50 cells, previously generated and decoded with OpenCV.
# 0 = black; 1 = white. Includes the mandatory one-cell black border.
$markerRows = @{
    1 = @('00000000','00000110','01011110','01011100','01000110','01000100','00100010','00000000')
    0 = @('00000000','00001110','01000110','01101110','00110000','00010100','01001100','00000000')
}

$culture = [System.Globalization.CultureInfo]::InvariantCulture
function Number([double]$value) { $value.ToString('0.#####', $culture) }
$encoding = [System.Text.Encoding]::ASCII
foreach ($size in @(80, 24)) {
    $objects = [System.Collections.Generic.List[string]]::new()
    $objects.Add('<< /Type /Catalog /Pages 2 0 R >>')
    $objects.Add('<< /Type /Pages /Kids [4 0 R 6 0 R] /Count 2 >>')
    $objects.Add('<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>')
    $pageIndex = 0
    foreach ($id in @(1, 0)) {
        $mm = 72.0 / 25.4
        $cell = $size * $mm / 8
        $left = (210 - $size) * $mm / 2
        $bottom = (297 - $size) * $mm / 2
        $content = [System.Text.StringBuilder]::new()
        [void]$content.Append("1 g 0 0 $(Number (210*$mm)) $(Number (297*$mm)) re f`n0 g`n")
        for ($y = 0; $y -lt 8; $y++) { for ($x = 0; $x -lt 8; $x++) {
            if ($markerRows[$id][$y][$x] -eq '0') {
                [void]$content.Append("$(Number ($left+$x*$cell)) $(Number ($bottom+(7-$y)*$cell)) $(Number $cell) $(Number $cell) re f`n")
            }
        } }
        [void]$content.Append("BT /F1 16 Tf 40 790 Td (DICT_6X6_50 - ID $id) Tj ET`n")
        [void]$content.Append("BT /F1 12 Tf 40 765 Td (Black marker outer edge: $size mm at Actual size.) Tj ET`n")
        [void]$content.Append("BT /F1 11 Tf 40 60 Td (Keep at least $(Number ($size/8)) mm white margin on every edge.) Tj ET`n")
        [void]$content.Append("BT /F1 11 Tf 40 42 Td (Remeasure marker centers after mounting if resizing or relocating.) Tj ET`n")
        $stream = $content.ToString()
        $objects.Add("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 $(Number (210*$mm)) $(Number (297*$mm))] /Resources << /Font << /F1 3 0 R >> >> /Contents $(5+$pageIndex*2) 0 R >>")
        $objects.Add("<< /Length $($encoding.GetByteCount($stream)) >>`nstream`n${stream}endstream")
        $pageIndex++
    }
    $pdf = [System.Text.StringBuilder]::new("%PDF-1.4`n")
    $offsets = [System.Collections.Generic.List[int]]::new()
    for ($i = 0; $i -lt $objects.Count; $i++) {
        $offsets.Add($encoding.GetByteCount($pdf.ToString()))
        [void]$pdf.Append("$($i+1) 0 obj`n$($objects[$i])`nendobj`n")
    }
    $xref = $encoding.GetByteCount($pdf.ToString())
    [void]$pdf.Append("xref`n0 $($objects.Count+1)`n0000000000 65535 f `n")
    foreach ($offset in $offsets) { [void]$pdf.Append("$($offset.ToString('0000000000')) 00000 n `n") }
    [void]$pdf.Append("trailer`n<< /Size $($objects.Count+1) /Root 1 0 R >>`nstartxref`n$xref`n%%EOF`n")
    $path = Join-Path $OutputDirectory "DICT_6X6_50_IDs_1_and_0_${size}mm.pdf"
    [System.IO.File]::WriteAllBytes($path, $encoding.GetBytes($pdf.ToString()))
    Write-Output "Created $path"
}
