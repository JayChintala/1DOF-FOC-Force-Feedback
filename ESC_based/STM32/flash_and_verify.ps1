<#
.SYNOPSIS
  Build both ESC_based node firmwares, flash each B-G431B-ESC1 by ST-LINK
  serial, then run bus_check.py on the Pi over SSH and judge the result.

.DESCRIPTION
  Board mapping (verified 2026-11-18, see notes.md section 6):
    ESC1  CAN base 0x000  config Debug       (CAN_NODE_ID 0)  SN 066BFF525266555067073237  COM3
    ESC2  CAN base 0x020  config Debug_ESC2  (CAN_NODE_ID 1)  SN 066BFF515266555067014734  COM6

  Build: CubeIDE-generated makefiles + the CubeIDE-bundled GNU toolchain and
  make (headless, no GUI). The makefiles are git-ignored and NOT regenerated
  here: a new .c file must be added by hand to <cfg>/Application/User/subdir.mk
  and <cfg>/objects.list (or build once in the CubeIDE GUI).
  Never regenerate from MC Workbench / CubeMX.

  Verification: bus_check.py's own exit code is always 1 right after a flash
  (a freshly reset board reads ~+28 A idle = "RESET"), so this script parses
  its per-node lines instead:
    FAIL  a node's TELEM is missing/slow, or DBG is missing
    FAIL  a board that read "calibrated" before the flash does not read RESET
          after it, or an unflashed calibrated board reads RESET
          (= serial -> node mapping is wrong)
    WARN  µs-gaps outside 950..1050 (HAT frame drops cause these too)
    WARN  Pi throttled != 0x0
  The mapping is only confirmed from the bus when ONE board is flashed
  (-Board ESC1|ESC2) and both were calibrated beforehand: the flashed one
  must reset, the other must not. Flashing both, or flashing boards that
  are already RESET, can't tell the serials apart; the script says so.
  After a flash both boards need zero_iq_check.py (energizes, 0 A, arms free,
  user's go) before any teleop. This script never energizes a motor.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File flash_and_verify.ps1
  powershell -ExecutionPolicy Bypass -File flash_and_verify.ps1 -BuildOnly
  powershell -ExecutionPolicy Bypass -File flash_and_verify.ps1 -Board ESC2 -Clean
  powershell -ExecutionPolicy Bypass -File flash_and_verify.ps1 -VerifyOnly
#>
[CmdletBinding()]
param(
    [ValidateSet('both', 'ESC1', 'ESC2')] [string]$Board = 'both',
    [switch]$BuildOnly,      # build, don't flash or check
    [switch]$NoBuild,        # flash the existing ELFs
    [switch]$VerifyOnly,     # only run bus_check on the Pi
    [switch]$Clean,          # make clean before building
    [switch]$Yes,            # skip the "type FLASH" confirmation
    [switch]$DryRun,         # print the flash commands, don't run them
    [double]$CheckSeconds = 3.0,
    [string]$PiHost = 'jayrpi4@192.168.1.164',
    [string]$PiSoftwareDir = '~/1DOF-FOC-Force-Feedback/ESC_based/software',
    [string]$CubeIdeRoot = 'C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE'
)

$ErrorActionPreference = 'Stop'
$ProjDir = Join-Path $PSScriptRoot 'MCWorkbench\STM32CubeIDE'

$Boards = [ordered]@{
    ESC1 = @{ Cfg = 'Debug';      NodeId = 0; Node = 'A'; Sn = '066BFF525266555067073237'; Com = 'COM3' }
    ESC2 = @{ Cfg = 'Debug_ESC2'; NodeId = 1; Node = 'B'; Sn = '066BFF515266555067014734'; Com = 'COM6' }
}
$Selected = if ($Board -eq 'both') { @('ESC1', 'ESC2') } else { @($Board) }

# ---------------------------------------------------------------- logging
$LogDir = Join-Path $PSScriptRoot 'flash_logs'
New-Item -ItemType Directory -Force $LogDir | Out-Null
$LogFile = Join-Path $LogDir ("flash_{0:yyyyMMdd_HHmmss}.log" -f (Get-Date))
function Log([string]$msg, [string]$color = '') {
    if ($color) { Write-Host $msg -ForegroundColor $color } else { Write-Host $msg }
    Add-Content -Path $LogFile -Value $msg -Encoding utf8
}
function Die([string]$msg, [int]$code) { Log "FAIL: $msg" Red; Log "log: $LogFile"; exit $code }

# Run a native command through cmd so stderr merges without PowerShell
# ErrorRecords; returns @{Code; Out}.
function Invoke-Native([string]$cmdline) {
    $out = cmd /c "$cmdline 2>&1"
    return @{ Code = $LASTEXITCODE; Out = ($out -join "`n") }
}

# ---------------------------------------------------------------- tools
function Find-Plugin([string]$pattern) {
    $d = Get-ChildItem (Join-Path $CubeIdeRoot 'plugins') -Directory -Filter $pattern -ErrorAction SilentlyContinue |
         Sort-Object Name -Descending | Select-Object -First 1
    if ($d) { return (Join-Path $d.FullName 'tools\bin') }
    return $null
}

function Get-ProgrammerCli {
    $c = Get-Command STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
    $p = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    if (Test-Path $p) { return $p }
    $b = Find-Plugin 'com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_*'
    if ($b -and (Test-Path (Join-Path $b 'STM32_Programmer_CLI.exe'))) { return (Join-Path $b 'STM32_Programmer_CLI.exe') }
    Die 'STM32_Programmer_CLI.exe not found' 4
}

# ---------------------------------------------------------------- build
function Build-All {
    $gcc  = Find-Plugin 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*.win32_*'
    $make = Find-Plugin 'com.st.stm32cube.ide.mcu.externaltools.make.win32_*'
    if (-not $gcc -or -not $make) { Die "CubeIDE toolchain/make not found under $CubeIdeRoot\plugins" 2 }
    $env:PATH = "$gcc;$make;$env:PATH"

    foreach ($b in $Selected) {
        $cfg = $Boards[$b].Cfg
        $cfgDir = Join-Path $ProjDir $cfg
        if (-not (Test-Path (Join-Path $cfgDir 'makefile'))) {
            Die "$cfgDir\makefile missing: build $cfg once in the CubeIDE GUI to generate it" 2
        }
        # Guard: the node ID is compile-time. Debug_ESC2 must define 1, Debug must not define it (header default 0).
        $mk = Get-Content (Join-Path $cfgDir 'Application\User\subdir.mk') -Raw
        $want = $Boards[$b].NodeId
        $has1 = $mk -match '-DCAN_NODE_ID=1\b'
        $hasAny = $mk -match '-DCAN_NODE_ID='
        if (($want -eq 1 -and -not $has1) -or ($want -eq 0 -and $hasAny)) {
            Die "$cfg makefiles do not match CAN_NODE_ID $want (check C/C++ Build > Settings > Preprocessor)" 2
        }

        Log "--- build $b ($cfg, CAN_NODE_ID $want)" Cyan
        if ($Clean) { Invoke-Native "make -C `"$cfgDir`" clean" | Out-Null }
        $t0 = Get-Date
        $r = Invoke-Native "make -j8 -C `"$cfgDir`" all"
        Add-Content -Path $LogFile -Value $r.Out -Encoding utf8
        $warn = ($r.Out -split "`n" | Where-Object { $_ -match 'warning:' })
        $err  = ($r.Out -split "`n" | Where-Object { $_ -match 'error' -and $_ -notmatch '-Werror|\.cyclo|\.su' })
        if ($r.Code -ne 0) { $err | Select-Object -First 20 | ForEach-Object { Log $_ Red }; Die "make $cfg exited $($r.Code)" 2 }
        $elf = Join-Path $cfgDir 'ESC_based.elf'
        $size = Invoke-Native "arm-none-eabi-size `"$elf`""
        Log ($size.Out -split "`n" | Select-Object -Last 1)
        $built = if ((Get-Item $elf).LastWriteTime -ge $t0) { 'relinked' } else { 'up to date' }
        Log ("{0} {1}, {2} warning(s), {3:N1} s" -f $elf, $built, @($warn).Count, ((Get-Date) - $t0).TotalSeconds) Green
        if (@($warn).Count) { $warn | Select-Object -First 10 | ForEach-Object { Log "  $_" Yellow } }
    }
}

# ---------------------------------------------------------------- Pi check
function Invoke-BusCheck([string]$label) {
    Log "--- bus_check.py on the Pi ($label, ${CheckSeconds} s, passive)" Cyan
    $r = Invoke-Native "ssh -o BatchMode=yes -o ConnectTimeout=8 $PiHost `"cd $PiSoftwareDir && python3 bus_check.py --time $CheckSeconds`""
    $r.Out -split "`n" | ForEach-Object { Log "  $_" }
    if ($r.Code -eq 255) { Die "ssh to $PiHost failed" 4 }
    $nodes = @{}
    foreach ($line in ($r.Out -split "`n")) {
        if ($line -match '^(A|B) \(.*?\): TELEM (\d+)/s(?:, DBG (\d+)/s, us-gaps outside 950\.\.1050: (\d+), idle Iq mean ([+-]?[\d.]+) A \((RESET|calibrated))?') {
            $nodes[$Matches[1]] = @{
                Telem = [int]$Matches[2]
                Dbg   = if ($Matches[3]) { [int]$Matches[3] } else { 0 }
                Gaps  = if ($Matches[4]) { [int]$Matches[4] } else { -1 }
                Iq    = if ($Matches[5]) { [double]$Matches[5] } else { [double]::NaN }
                State = if ($Matches[6]) { $Matches[6] } else { 'MISSING' }
            }
        }
    }
    $thr = if ($r.Out -match 'throttled=(0x[0-9a-fA-F]+)') { $Matches[1] } else { '?' }
    return @{ Nodes = $nodes; Throttled = $thr }
}

# ---------------------------------------------------------------- main
$gitRev = (git -C $PSScriptRoot rev-parse --short HEAD) 2>$null
$dirty  = (git -C $PSScriptRoot status --porcelain -- MCWorkbench) 2>$null
Log ("flash_and_verify  {0:yyyy-MM-dd HH:mm:ss}  boards={1}  firmware git {2}{3}" -f (Get-Date), ($Selected -join ','), $gitRev, $(if ($dirty) { ' (STM32 tree DIRTY)' } else { '' }))

if ($VerifyOnly) {
    $post = Invoke-BusCheck 'verify only'
    $BuildOnly = $false; $Selected = @()
} else {
    if (-not $NoBuild) { Build-All }
    if ($BuildOnly) { Log "Build only: done. log: $LogFile" Green; exit 0 }

    # Preflight: probes present
    $cli = Get-ProgrammerCli
    $probes = (Invoke-Native "`"$cli`" -l st").Out
    foreach ($b in $Selected) {
        if ($probes -notmatch $Boards[$b].Sn) { Die "$b ST-LINK $($Boards[$b].Sn) not connected" 4 }
    }
    $pre = Invoke-BusCheck 'before flash'

    Log ''
    Log 'About to flash:' Yellow
    foreach ($b in $Selected) {
        $i = $Boards[$b]
        Log ("  {0} (node {1}, CAN_NODE_ID {2}) SN {3} <- {4}\ESC_based.elf" -f $b, $i.Node, $i.NodeId, $i.Sn, $i.Cfg) Yellow
    }
    Log '  Both boards will reset; the motors are not energized. Afterwards they need zero_iq_check.py.' Yellow
    if (-not $Yes -and -not $DryRun) {
        $ans = Read-Host "Type FLASH to continue"
        if ($ans -ne 'FLASH') { Die 'aborted by user, nothing flashed' 5 }
    }

    foreach ($b in $Selected) {
        $i = $Boards[$b]
        $elf = Join-Path (Join-Path $ProjDir $i.Cfg) 'ESC_based.elf'
        if (-not (Test-Path $elf)) { Die "$elf missing" 3 }
        $cmd = "`"$cli`" -c port=SWD sn=$($i.Sn) -w `"$elf`" -v -rst"
        Log "--- flash $b" Cyan
        if ($DryRun) { Log "  [dry run] $cmd"; continue }
        $r = Invoke-Native $cmd
        Add-Content -Path $LogFile -Value $r.Out -Encoding utf8
        $verified = $r.Out -match 'Download verified successfully'
        if ($r.Code -ne 0 -or -not $verified) {
            $r.Out -split "`n" | Select-Object -Last 15 | ForEach-Object { Log "  $_" Red }
            Die "$b flash failed (exit $($r.Code), verified=$verified)" 3
        }
        Log "  $b written, verified, reset" Green
    }
    if ($DryRun) { Log "Dry run: done. log: $LogFile"; exit 0 }
    Start-Sleep -Seconds 2   # boards reboot in < 3 ms; settle CAN before listening
    $post = Invoke-BusCheck 'after flash'
}

# ---------------------------------------------------------------- verdict
$fail = @(); $warn = @(); $info = @()
foreach ($b in $Boards.Keys) {
    $n = $Boards[$b].Node
    $p = $post.Nodes[$n]
    if (-not $p -or $p.State -eq 'MISSING') { $fail += "$b (node $n): TELEM missing or slow"; continue }
    if ($p.Telem -lt 900) { $fail += "$b (node $n): TELEM $($p.Telem)/s" }
    if ($p.Dbg -le 0)     { $fail += "$b (node $n): no DBG frames" }
    if ($p.Gaps -gt 0)    { $warn += "$b (node $n): $($p.Gaps) us-gaps off the 1 ms grid (HAT drops or a reset during the listen; rerun -VerifyOnly)" }
    if ($pre) {
        $was = if ($pre.Nodes[$n]) { $pre.Nodes[$n].State } else { 'MISSING' }
        $flashed = $Selected -contains $b
        if ($flashed -and $was -eq 'calibrated' -and $p.State -ne 'RESET') {
            $fail += "$b (node $n) was flashed but did not reset: serial -> node mapping wrong?"
        } elseif (-not $flashed -and $was -eq 'calibrated' -and $p.State -eq 'RESET') {
            $fail += "$b (node $n) was NOT flashed but reset: serial -> node mapping wrong?"
        } elseif ($flashed -and $was -eq 'calibrated' -and $Selected.Count -gt 1) {
            $info += "$b (node $n): calibrated -> RESET as expected (both boards flashed, so this can't tell the serials apart; flash one with -Board to confirm the mapping)"
        } elseif ($flashed -and $was -eq 'calibrated') {
            $info += "$b (node $n): calibrated -> RESET, mapping confirmed on the bus"
        } elseif ($flashed) {
            $info += "$b (node $n): was already $was before the flash, so the bus can't confirm the mapping (presence + ID still checked)"
        }
    }
}
if ($post.Throttled -ne '0x0') { $warn += "Pi throttled=$($post.Throttled) (want 0x0; 0x50000 = under-voltage/soft-temp occurred since boot)" }
if (@($post.Nodes.Values | Where-Object { $_.State -eq 'RESET' }).Count) {
    $info += 'RESET boards need zero_iq_check.py (energizes at 0 A: user''s go, both arms free) before teleop'
}

Log ''
$info | ForEach-Object { Log "INFO  $_" }
$warn | ForEach-Object { Log "WARN  $_" Yellow }
$fail | ForEach-Object { Log "FAIL  $_" Red }
if ($fail.Count) { Log "RESULT: FAIL  (log: $LogFile)" Red; exit 1 }
Log "RESULT: PASS  (log: $LogFile)" Green
exit 0
