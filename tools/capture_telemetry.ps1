param(
    [Parameter(Mandatory=$true)][string]$Port,
    [Parameter(Mandatory=$true)][string]$Output,
    [ValidateRange(1,86400)][int]$Seconds=60
)
$ErrorActionPreference='Stop'
$capturePath=$ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Output)
if(Test-Path -LiteralPath $capturePath){throw "Output already exists: $capturePath"}
$serialPort=[System.IO.Ports.SerialPort]::new($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serialPort.ReadTimeout=200
$serialPort.DtrEnable=$false
$serialPort.RtsEnable=$false
$captureStream=$null
try {
    $serialPort.Open()
    $captureStream=[System.IO.File]::Open($capturePath,[System.IO.FileMode]::CreateNew,[System.IO.FileAccess]::Write)
    $captureBuffer=New-Object byte[] 4096
    $captureTimer=[System.Diagnostics.Stopwatch]::StartNew()
    $captureBytes=0
    while($captureTimer.Elapsed.TotalSeconds -lt $Seconds){
        try {
            $captureCount=$serialPort.Read($captureBuffer,0,$captureBuffer.Length)
            $captureStream.Write($captureBuffer,0,$captureCount)
            $captureBytes+=$captureCount
        } catch [System.TimeoutException] { }
    }
    Write-Output "Saved $captureBytes bytes to $capturePath"
} finally {
    if($null -ne $captureStream){$captureStream.Dispose()}
    $serialPort.Dispose()
}
