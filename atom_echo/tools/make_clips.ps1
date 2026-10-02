# Regenerates the Atom Echo's spoken clips with Windows text-to-speech, then
# packs them into src/Clips.h. Run from the atom_echo folder:
#   powershell -ExecutionPolicy Bypass -File tools\make_clips.ps1
# Edit the phrases here, never Clips.h.

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Speech

$phrases = [ordered]@{
  ready           = 'Echo ready'
  logger_found    = 'Logger connected'
  logger_lost     = 'Logger link lost'
  no_logger       = 'No logger'
  fix_ok          = 'GPS fix'
  fix_lost        = 'GPS fix lost'
  no_fix          = 'No GPS fix'
  saved           = 'Waypoint saved'
  rejected        = 'Waypoint not saved'
  no_confirm      = 'No confirmation'
  atom_ok         = 'Atom linked'
  atom_lost       = 'Atom link lost'
  sd_error        = 'SD card error'
  sd_ok           = 'SD card ready'
  battery_low     = 'Logger battery low'
  battery_critical = 'Logger battery critical'
  muted           = 'Alerts muted'
  unmuted         = 'Alerts on'
  trex_logging    = 'T-Rex says, rawr! Is logging.'
  trex_not_logging = 'T-Rex says, rawr! Not logging.'
  pair_adv        = 'Pairs with Cardputer A D V'
  pair_cardputer  = 'Pairs with Cardputer'
  pair_core2      = 'Pairs with Core 2'
  pair_any        = 'Pairs with any logger'
}

$root = Split-Path -Parent $PSScriptRoot
$wavDir = Join-Path $root 'clips'
New-Item -ItemType Directory -Force $wavDir | Out-Null

$voice = New-Object System.Speech.Synthesis.SpeechSynthesizer
$voice.SelectVoice('Microsoft Zira Desktop')
# Slightly slower than normal: clearer through a small speaker.
$voice.Rate = -1
$format = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(
  16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
  [System.Speech.AudioFormat.AudioChannel]::Mono)

foreach ($name in $phrases.Keys) {
  $path = Join-Path $wavDir "$name.wav"
  $voice.SetOutputToWaveFile($path, $format)
  $voice.Speak($phrases[$name])
}
$voice.SetOutputToNull()
$voice.Dispose()

python (Join-Path $PSScriptRoot 'pack_clips.py') $wavDir (Join-Path $root 'src\Clips.h') @($phrases.Keys)
