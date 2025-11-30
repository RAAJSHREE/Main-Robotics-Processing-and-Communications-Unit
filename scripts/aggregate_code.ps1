param(
    [string]$Root = "C:\Project\Main-Robotics-Processing-and-Communications-Unit\Main-Robotics-Processing-and-Communications-Unit",
    [string]$Dest = "$Root\ALL_CODE.txt"
)

Write-Host "Aggregating source files under: $Root"
Remove-Item $Dest -ErrorAction SilentlyContinue

$extensions = @('.py', '.cpp', '.h', '.html', '.js', '.yml', '.yaml', '.conf')

Get-ChildItem -Path $Root -Recurse -File -ErrorAction SilentlyContinue |
    Where-Object { ($_.Extension -in $extensions) -or ($_.Name -eq 'Dockerfile') } |
    Where-Object { $_.FullName -notmatch "\\build\\|\\install\\|\\__pycache__\\" } |
    ForEach-Object {
        Add-Content -Path $Dest -Value "`n=== FILE: $($_.FullName) ===`n"
        try {
            Get-Content $_.FullName -ErrorAction Stop | Add-Content -Path $Dest
        }
        catch {
            Add-Content -Path $Dest -Value "[ERROR READING FILE: $($_.FullName)]"
        }
    }

Write-Host "Done. Output: $Dest"