# 一键把本地分支推送到组仓库（会自动判断远程是否已有代码）
# 用法：双击同目录的 push_to_github.bat

$ErrorActionPreference = 'Stop'
$repoUrl = 'https://github.com/windbell0711/DTD.git'
$branch  = 'feature/chassis-kinematics'
$baseCommit = '145d361'   # 未修改框架的基线

$project = Split-Path $PSScriptRoot -Parent
Set-Location $project

if (-not (Test-Path (Join-Path $project '.git'))) {
    throw "这里不是 git 仓库：$project"
}

Write-Host "工程目录：$project"
Write-Host "目标仓库：$repoUrl"
Write-Host "目标分支：$branch"
Write-Host ""

# 确保本地有这个分支
$localBranches = git branch --format='%(refname:short)'
if ($localBranches -notcontains $branch) {
    Write-Host "本地没有 $branch，从当前提交新建..."
    git checkout -b $branch
}

Write-Host "== 1/3 检查远程仓库（会弹浏览器登录 GitHub，请授权） =="
$heads = & git ls-remote --heads $repoUrl 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host $heads
    throw "无法访问仓库。请确认地址正确，并在弹出的浏览器里完成登录。"
}

if (-not $heads) {
    Write-Host "远程是空仓库 → 直接推送本地分支"
    git remote remove origin 2>$null
    git remote add origin $repoUrl
    git push -u origin $branch
    if ($LASTEXITCODE -ne 0) { throw "推送失败，请把上面的红字发给我" }
} else {
    Write-Host "远程已有代码 → 用补丁方式，把我们的改动应用到新分支上"
    $tmp = Join-Path $env:TEMP ("DTD-clone-" + (Get-Random))
    git clone $repoUrl $tmp
    if ($LASTEXITCODE -ne 0) { throw "克隆失败，请把上面的红字发给我" }

    $patch = Join-Path $env:TEMP ("chassis-kinematics-" + (Get-Random) + ".patch")
    git diff --output=$patch $baseCommit HEAD -- . ':(exclude)MDK-ARM/startup_stm32f407xx.lst' ':(exclude)MDK-ARM/RM_Frame_C.uvoptx'

    Push-Location $tmp
    try {
        git checkout -b $branch
        git apply $patch
        if ($LASTEXITCODE -ne 0) {
            throw "补丁应用失败：远程代码和我们本地的基线不一致（可能队友已经改过同一批文件）"
        }
        git add -A
        git -c user.name="$env:USERNAME" -c user.email="$env:USERNAME@users.noreply.github.com" `
            commit -m "feat: 底盘运动学 ChassisKinematics + 遥控判档/中位校准 + 调试工具"
        git push -u origin $branch
        if ($LASTEXITCODE -ne 0) { throw "推送失败，请把上面的红字发给我" }
    } finally {
        Pop-Location
        Remove-Item $patch -Force -ErrorAction SilentlyContinue
        Write-Host "临时克隆保留在：$tmp（确认无误后可手动删除）"
    }
}

Write-Host ""
Write-Host "== 2/3 完成 ==" -ForegroundColor Green
Write-Host "打开下面链接创建 Pull Request（base 选 main，compare 选 $branch）："
Write-Host ("{0}/compare/main...{1}?expand=1" -f ($repoUrl -replace '\.git$', ''), $branch)
