<#
  Run a command on the Pi over SSH and speak every "PHASE n: text" line it
  prints, so a person at the bench hears which phase is running.
  Usage: powershell -File run_announced.ps1 "python3 listen_test.py"
#>
param([Parameter(Mandatory)] [string]$RemoteCmd,
      [string]$PiHost = 'jayrpi4@192.168.1.164',
      [string]$Dir = '~/1DOF-FOC-Force-Feedback/ESC_based/software')
Add-Type -AssemblyName System.Speech
$tts = New-Object System.Speech.Synthesis.SpeechSynthesizer
$tts.Rate = 1
cmd /c "ssh -o BatchMode=yes $PiHost `"cd $Dir && $RemoteCmd`" 2>&1" | ForEach-Object {
    Write-Host $_
    if ($_ -match '^PHASE \d+: (.*)$') { $tts.SpeakAsync($Matches[1]) | Out-Null }
    elseif ($_ -match '^\s*(Leader cal|>>>|Teleop|GO|Calibration)') { $tts.SpeakAsync($_) | Out-Null }
}
Start-Sleep -Milliseconds 1500
