# Start the HopeJR Desk frontend dev server on Windows.
$ErrorActionPreference = "Stop"
Set-Location "$PSScriptRoot\..\frontend"
if (-not (Test-Path node_modules)) { npm install }
npm run dev -- --host
