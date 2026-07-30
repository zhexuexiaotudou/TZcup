param(
    [Parameter(Mandatory = $true)][string]$DataRoot
)

$ErrorActionPreference = "Stop"
$resolved = (Resolve-Path -LiteralPath $DataRoot).Path
$manifest = Get-Content -LiteralPath (Join-Path $resolved "speech_manifest.json") -Raw -Encoding utf8 | ConvertFrom-Json
$rawRoot = Join-Path $resolved "raw"
New-Item -ItemType Directory -Force -Path $rawRoot | Out-Null

Add-Type -AssemblyName System.Speech
$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
$format = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(
    16000,
    [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
    [System.Speech.AudioFormat.AudioChannel]::Mono
)
try {
    foreach ($case in $manifest.cases) {
        $path = Join-Path $resolved $case.raw_audio
        $synth.SelectVoice([string]$case.voice)
        $synth.Rate = [int]$case.speech_rate
        $synth.SetOutputToWaveFile($path, $format)
        $synth.Speak([string]$case.text)
        $synth.SetOutputToNull()
    }
}
finally {
    $synth.Dispose()
}
