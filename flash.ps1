# Helper script to flash Bootloader and Application using ST-LINK and STM32CubeProgrammer CLI

$ErrorActionPreference = "Stop"

# Common installation paths for STM32_Programmer_CLI.exe
$cliPaths = @(
    "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
    "C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)

# Also search STM32CubeIDE plugins directory if installed there
$cubeIdeBase = "C:\ST\STM32CubeIDE"
if (Test-Path $cubeIdeBase) {
    $foundClis = Get-ChildItem -Path $cubeIdeBase -Filter "STM32_Programmer_CLI.exe" -Recurse -ErrorAction SilentlyContinue
    foreach ($cli in $foundClis) {
        $cliPaths += $cli.FullName
    }
}

# Find the first path that exists
$programmerCli = $null
foreach ($path in $cliPaths) {
    if (Test-Path $path) {
        $programmerCli = $path
        break
    }
}

# If not found, try to locate it in system PATH
if (-not $programmerCli) {
    $programmerCli = Get-Command "STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source
}

if (-not $programmerCli) {
    Write-Error "STM32_Programmer_CLI.exe not found. Please install STM32CubeProgrammer or add it to your PATH."
    exit 1
}

Write-Host "Found STM32CubeProgrammer CLI at: $programmerCli" -ForegroundColor Green

# Define firmware paths
$bootloaderElf = Join-Path $PSScriptRoot "Boot\build\Lumon_Bot_MainController_Firmware_Boot.elf"
$applicationElf = Join-Path $PSScriptRoot "Appli\build\Lumon_Bot_MainController_Firmware_Appli.elf"

# Check if binaries exist
if (-not (Test-Path $bootloaderElf)) {
    Write-Warning "Bootloader binary not found at $bootloaderElf. Have you built the Boot project?"
}
if (-not (Test-Path $applicationElf)) {
    Write-Warning "Application binary not found at $applicationElf. Have you built the Appli project?"
}

function Flash-Binary {
    param(
        [string]$filePath,
        [string]$address
    )
    
    if (-not (Test-Path $filePath)) {
        Write-Error "File not found: $filePath"
        return
    }

    Write-Host "--------------------------------------------------" -ForegroundColor Cyan
    Write-Host "Flashing $filePath to address $address..." -ForegroundColor Cyan
    Write-Host "--------------------------------------------------" -ForegroundColor Cyan

    $cmdArgs = @("-c", "port=SWD", "mode=UR")

    # If programming external memory (0x90000000)
    if ($address -eq "0x90000000") {
        # Convert ELF to BIN if needed to avoid RAM segments issue
        if ($filePath.EndsWith(".elf")) {
            $binPath = $filePath.Replace(".elf", ".bin")
            $objcopy = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin\arm-none-eabi-objcopy.exe"
            
            Write-Host "Converting ELF to BIN to strip RAM segments..." -ForegroundColor Yellow
            & $objcopy -O binary $filePath $binPath
            $filePath = $binPath
        }

        # Reference our custom loader file if built, otherwise fallback to the default ST one
        $loaderElf = Join-Path $PSScriptRoot "ExtMemLoader\build\Lumon_Bot_MainController_Firmware_ExtMemLoader.elf"
        $loaderStldr = Join-Path $PSScriptRoot "ExtMemLoader\build\Lumon_Bot_MainController_Firmware_ExtMemLoader.stldr"
        
        if (Test-Path $loaderElf) {
            # Copy elf to stldr since STM32_Programmer_CLI requires the .stldr extension
            Copy-Item -Path $loaderElf -Destination $loaderStldr -Force
            $loaderName = $loaderStldr
            Write-Host "Using custom external loader: $loaderName" -ForegroundColor Green
        } else {
            $loaderName = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\ExternalLoader\MX25UW25645G_NUCLEO-H7S3L8-XSPIM1.stldr"
        }
        $cmdArgs += @("-el", $loaderName)
    }

    # Add program and verify commands
    $cmdArgs += @("-w", $filePath, $address, "-v")

    # Connect via ST-LINK (SWD), erase and program, then verify
    & $programmerCli $cmdArgs
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Successfully flashed and verified!" -ForegroundColor Green
    }
    else {
        Write-Error "Flashing failed with exit code $LASTEXITCODE"
    }
}

# Prompt user for choices
Write-Host "Select what to flash:"
Write-Host "1) Flash Bootloader only (to 0x08000000)"
Write-Host "2) Flash Application only (to 0x90000000)"
Write-Host "3) Flash Both (Bootloader to 0x08000000, Appli to 0x90000000 - WARNING: Overwrites if both target 0x08000000)"
$choice = Read-Host "Enter option (1, 2, or 3)"

switch ($choice) {
    "1" {
        Flash-Binary -filePath $bootloaderElf -address "0x08000000"
    }
    "2" {
        Flash-Binary -filePath $applicationElf -address "0x90000000"
    }
    "3" {
        Flash-Binary -filePath $bootloaderElf -address "0x08000000"
        Flash-Binary -filePath $applicationElf -address "0x90000000"
    }
    default {
        Write-Host "Invalid choice. Exiting." -ForegroundColor Red
    }
}
