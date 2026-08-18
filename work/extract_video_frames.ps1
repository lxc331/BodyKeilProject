param(
    [Parameter(Mandatory = $true)]
    [string]$VideoPath,

    [Parameter(Mandatory = $true)]
    [string]$OutputDir,

    [double[]]$Times = @()
)

$ErrorActionPreference = 'Stop'

Add-Type -AssemblyName PresentationCore
Add-Type -AssemblyName WindowsBase

New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

$player = New-Object System.Windows.Media.MediaPlayer
$script:openedFlag = $false
$script:failedFlag = $false
$script:openError = $null

$openedHandler = [System.EventHandler] { $script:openedFlag = $true }
$failedHandler = [System.EventHandler[System.Windows.Media.ExceptionEventArgs]] {
    param($sender, $eventArgs)
    $script:openError = $eventArgs.ErrorException
    $script:failedFlag = $true
}

function Invoke-DispatcherFor([int]$Milliseconds) {
    $deadline = [DateTime]::UtcNow.AddMilliseconds($Milliseconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        [System.Windows.Threading.Dispatcher]::CurrentDispatcher.Invoke(
            [System.Action]{},
            [System.Windows.Threading.DispatcherPriority]::Background
        )
        Start-Sleep -Milliseconds 15
    }
}

$player.add_MediaOpened($openedHandler)
$player.add_MediaFailed($failedHandler)
$player.Volume = 0
$player.ScrubbingEnabled = $true
$player.Open([Uri]::new((Resolve-Path -LiteralPath $VideoPath).Path))

$openDeadline = [DateTime]::UtcNow.AddSeconds(15)
while (-not $script:openedFlag -and -not $script:failedFlag -and [DateTime]::UtcNow -lt $openDeadline) {
    Invoke-DispatcherFor 50
}
if (-not $script:openedFlag -and -not $script:failedFlag) {
    throw "Timed out while opening video: $VideoPath"
}
if ($script:failedFlag) {
    throw "Failed to open video: $script:openError"
}

$width = $player.NaturalVideoWidth
$height = $player.NaturalVideoHeight
$duration = $player.NaturalDuration.TimeSpan.TotalSeconds
if ($width -le 0 -or $height -le 0) {
    throw "Video opened without a valid frame size"
}

if ($Times.Count -eq 0) {
    $Times = [double[]]@(
        0.0,
        ($duration * 0.25),
        ($duration * 0.5),
        ($duration * 0.75),
        [Math]::Max(0.0, ($duration - 0.2))
    )
}

foreach ($time in $Times) {
    $safeTime = [Math]::Min([Math]::Max(0.0, $time), [Math]::Max(0.0, ($duration - 0.05)))
    $player.Position = [TimeSpan]::FromSeconds($safeTime)
    $player.Play()
    Invoke-DispatcherFor 300
    $player.Pause()
    Invoke-DispatcherFor 120

    $visual = New-Object System.Windows.Media.DrawingVisual
    $context = $visual.RenderOpen()
    $rect = New-Object System.Windows.Rect(0, 0, $width, $height)
    $context.DrawVideo($player, $rect)
    $context.Close()

    $bitmap = New-Object System.Windows.Media.Imaging.RenderTargetBitmap(
        $width,
        $height,
        96,
        96,
        [System.Windows.Media.PixelFormats]::Pbgra32
    )
    $bitmap.Render($visual)

    $encoder = New-Object System.Windows.Media.Imaging.PngBitmapEncoder
    $encoder.Frames.Add([System.Windows.Media.Imaging.BitmapFrame]::Create($bitmap))
    $fileName = 'frame-{0:D6}.png' -f [int][Math]::Round($safeTime * 1000)
    $outputPath = Join-Path $OutputDir $fileName
    $stream = [System.IO.File]::Open($outputPath, [System.IO.FileMode]::Create)
    try {
        $encoder.Save($stream)
    }
    finally {
        $stream.Dispose()
    }
}

$player.Close()
$player.remove_MediaOpened($openedHandler)
$player.remove_MediaFailed($failedHandler)

[pscustomobject]@{
    Video = (Resolve-Path -LiteralPath $VideoPath).Path
    DurationSeconds = $duration
    Width = $width
    Height = $height
    FramesWritten = $Times.Count
    OutputDir = (Resolve-Path -LiteralPath $OutputDir).Path
}
