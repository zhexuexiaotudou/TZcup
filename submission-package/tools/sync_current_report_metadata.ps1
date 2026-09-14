$path = Join-Path $PSScriptRoot "..\校验记录.md"
$text = Get-Content -LiteralPath $path -Raw -Encoding UTF8
$text = $text.Replace("实际重建23页PDF", "实际重建24页PDF")
$text = $text.Replace("报告第1、22、23页", "报告第1、22、23、24页")
$text = $text.Replace("复核第1、22、23页", "复核第1、22、23、24页")
Set-Content -LiteralPath $path -Value $text -Encoding UTF8 -NoNewline
