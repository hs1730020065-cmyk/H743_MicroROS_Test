param(
  [Parameter(Mandatory = $true)]
  [string]$Port,

  [ValidateSet("center", "forward", "forward_low", "reverse", "left", "right")]
  [string]$Command = "forward_low",

  [int]$DurationMs = 3000,
  [int]$PeriodMs = 20
)

$frames = @{
  center      = "0F E0 03 1F F8 C0 37 71 56 80 0F 7C E0 03 1F F8 C0 07 3E F0 81 0F 7C 00 00"
  forward     = "0F E0 9B 38 F8 C0 37 71 56 80 0F 7C E0 03 1F F8 C0 07 3E F0 81 0F 7C 00 00"
  forward_low = "0F E0 83 25 F8 C0 37 71 56 80 0F 7C E0 03 1F F8 C0 07 3E F0 81 0F 7C 00 00"
  reverse     = "0F E0 63 05 F8 C0 37 71 56 80 0F 7C E0 03 1F F8 C0 07 3E F0 81 0F 7C 00 00"
  left        = "0F AC 00 1F F8 C0 37 71 56 80 0F 7C E0 03 1F F8 C0 07 3E F0 81 0F 7C 00 00"
  right       = "0F 13 07 1F F8 C0 37 71 56 80 0F 7C E0 03 1F F8 C0 07 3E F0 81 0F 7C 00 00"
}

$bytes = [byte[]]($frames[$Command].Split(" ") | ForEach-Object { [Convert]::ToByte($_, 16) })

$serial = New-Object System.IO.Ports.SerialPort
$serial.PortName = $Port
$serial.BaudRate = 100000
$serial.DataBits = 8
$serial.Parity = [System.IO.Ports.Parity]::Even
$serial.StopBits = [System.IO.Ports.StopBits]::Two

$serial.Open()
try {
  $deadline = [DateTime]::UtcNow.AddMilliseconds($DurationMs)
  while ([DateTime]::UtcNow -lt $deadline) {
    $serial.Write($bytes, 0, $bytes.Length)
    Start-Sleep -Milliseconds $PeriodMs
  }
}
finally {
  $serial.Close()
}
