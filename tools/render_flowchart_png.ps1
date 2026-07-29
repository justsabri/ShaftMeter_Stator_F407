param(
  [Parameter(Mandatory=$true)][string]$SpecPath,
  [Parameter(Mandatory=$true)][string]$OutPath
)

Add-Type -AssemblyName System.Drawing

$spec = Get-Content -Raw -Encoding UTF8 -Path $SpecPath | ConvertFrom-Json
$bmp = New-Object System.Drawing.Bitmap ([int]$spec.width), ([int]$spec.height)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::ClearTypeGridFit
$g.Clear([System.Drawing.Color]::White)

$linePen = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(85,85,85)), 2
$linePen.CustomEndCap = New-Object System.Drawing.Drawing2D.AdjustableArrowCap 5, 6
$rectPen = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(59,110,168)), 2
$rectBrush = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(247,250,252))
$textBrush = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(17,17,17))
$edgeBrush = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(51,51,51))
$font = New-Object System.Drawing.Font "Microsoft YaHei", 12
$edgeFont = New-Object System.Drawing.Font "Microsoft YaHei", 10
$sf = New-Object System.Drawing.StringFormat
$sf.Alignment = [System.Drawing.StringAlignment]::Center
$sf.LineAlignment = [System.Drawing.StringAlignment]::Center

foreach ($edge in $spec.edges) {
  $s = $spec.positions.($edge.src)
  $d = $spec.positions.($edge.dst)
  if ($null -eq $s -or $null -eq $d) { continue }
  $x1 = [float]($s.x + $spec.node_w / 2)
  $y1 = [float]($s.y + $spec.node_h)
  $x2 = [float]($d.x + $spec.node_w / 2)
  $y2 = [float]$d.y
  $midY = [float](($y1 + $y2) / 2)
  $path = New-Object System.Drawing.Drawing2D.GraphicsPath
  $path.AddBezier($x1, $y1, $x1, $midY, $x2, $midY, $x2, $y2)
  $g.DrawPath($linePen, $path)
  $path.Dispose()
  if ($edge.label -and $edge.label.Length -gt 0) {
    $labelRect = New-Object System.Drawing.RectangleF ([float](($x1 + $x2) / 2 - 60)), ([float]($midY - 18)), 120, 18
    $g.DrawString([string]$edge.label, $edgeFont, $edgeBrush, $labelRect, $sf)
  }
}

foreach ($node in $spec.nodes) {
  $p = $spec.positions.($node.id)
  if ($null -eq $p) { continue }
  $rect = New-Object System.Drawing.Rectangle ([int]$p.x), ([int]$p.y), ([int]$spec.node_w), ([int]$spec.node_h)
  if ($node.shape -eq "decision") {
    $cx = [float]($p.x + $spec.node_w / 2)
    $cy = [float]($p.y + $spec.node_h / 2)
    $pts = @(
      (New-Object System.Drawing.PointF $cx, ([float]$p.y)),
      (New-Object System.Drawing.PointF ([float]($p.x + $spec.node_w)), $cy),
      (New-Object System.Drawing.PointF $cx, ([float]($p.y + $spec.node_h))),
      (New-Object System.Drawing.PointF ([float]$p.x), $cy)
    )
    $g.FillPolygon($rectBrush, $pts)
    $g.DrawPolygon($rectPen, $pts)
  } else {
    $g.FillRectangle($rectBrush, $rect)
    $g.DrawRectangle($rectPen, $rect)
  }
  $textRect = New-Object System.Drawing.RectangleF ([float]$p.x), ([float]$p.y), ([float]$spec.node_w), ([float]$spec.node_h)
  $g.DrawString([string]$node.label, $font, $textBrush, $textRect, $sf)
}

$bmp.Save($OutPath, [System.Drawing.Imaging.ImageFormat]::Png)
$sf.Dispose()
$font.Dispose()
$edgeFont.Dispose()
$textBrush.Dispose()
$edgeBrush.Dispose()
$rectBrush.Dispose()
$rectPen.Dispose()
$linePen.Dispose()
$g.Dispose()
$bmp.Dispose()
