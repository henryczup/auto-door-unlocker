Add-Type -AssemblyName System.Drawing

function Resize-Image {
    param([string]$ImagePath, [string]$OutputPath, [int]$MaxWidth = 800)
    
    $img = [System.Drawing.Image]::FromFile($ImagePath)
    $ratio = $MaxWidth / $img.Width
    $newWidth = [int]($img.Width * $ratio)
    $newHeight = [int]($img.Height * $ratio)
    
    $bmp = New-Object System.Drawing.Bitmap($newWidth, $newHeight)
    $graph = [System.Drawing.Graphics]::FromImage($bmp)
    $graph.DrawImage($img, 0, 0, $newWidth, $newHeight)
    
    $bmp.Save($OutputPath, $img.RawFormat)
    
    $img.Dispose()
    $bmp.Dispose()
    $graph.Dispose()
}

$images = @(
    "media/image_67210241.JPG",
    "media/image_6209779 (4).JPG"
)

foreach ($image in $images) {
    $outputPath = $image -replace "\.JPG$", "_small.jpg"
    Resize-Image -ImagePath $image -OutputPath $outputPath
}
